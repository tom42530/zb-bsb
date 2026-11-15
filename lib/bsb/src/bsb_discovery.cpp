#if defined(ESP_PLATFORM)

#include <Arduino.h>
#include "bsb_discovery.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

namespace bsb {

static const char* TAG           = "bsb.discovery";
static const char* NVS_NAMESPACE = "zb_bsb";
static const char* NVS_KEY_CAPS  = "caps_v1";
static const char* NVS_KEY_RESCAN = "rescan_req";

static const char* ext_used_key(uint8_t zone_index) {
    return (zone_index == 2) ? "ext_used_z2" : "ext_used_z1";
}

static uint8_t read_flag(const char* key) {
    nvs_handle_t handle = 0;
    uint8_t      value  = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, key, &value);
        nvs_close(handle);
    }
    return value;
}

// N'écrit en flash que si la valeur change
static void write_flag(const char* key, uint8_t value) {
    if (read_flag(key) == value) {
        return;
    }
    nvs_handle_t handle = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    if (value != 0) {
        nvs_set_u8(handle, key, value);
    } else {
        nvs_erase_key(handle, key);
    }
    nvs_commit(handle);
    nvs_close(handle);
}

// Une valeur initiale illisible (NaN, hors plage) après corruption de la flash ne doit jamais
// atteindre les attributs Zigbee : on retombe sur une valeur par défaut plausible.
static float sanitize_temp(float value, float min_c, float max_c, float fallback_c) {
    return (std::isfinite(value) && value >= min_c && value <= max_c) ? value : fallback_c;
}

static void sanitize_zone(ZoneCapabilities& zone) {
    zone.initial_comfort_c = sanitize_temp(zone.initial_comfort_c, 5.0f, 35.0f, 20.0f);
    zone.initial_reduced_c = sanitize_temp(zone.initial_reduced_c, 5.0f, 35.0f, 18.0f);
    zone.initial_active_c  = sanitize_temp(zone.initial_active_c, 5.0f, 35.0f, zone.initial_comfort_c);
    zone.initial_room_c    = sanitize_temp(zone.initial_room_c, -5.0f, 45.0f, zone.initial_active_c);
    zone.initial_flow_c    = sanitize_temp(zone.initial_flow_c, 0.0f, 95.0f, 25.0f);
    if (zone.initial_mode > static_cast<uint8_t>(HeatingMode::Comfort)) {
        zone.initial_mode = static_cast<uint8_t>(HeatingMode::Auto);
    }
}

static void sanitize_capabilities(BsbCapabilities& caps) {
    caps.controller_model[sizeof(caps.controller_model) - 1] = '\0';
    caps.initial_outdoor_c      = sanitize_temp(caps.initial_outdoor_c, -50.0f, 70.0f, 15.0f);
    caps.initial_hp_flow_c      = sanitize_temp(caps.initial_hp_flow_c, 0.0f, 95.0f, 25.0f);
    caps.initial_hp_return_c    = sanitize_temp(caps.initial_hp_return_c, 0.0f, 95.0f, 25.0f);
    caps.initial_dhw_setpoint_c = sanitize_temp(caps.initial_dhw_setpoint_c, 25.0f, 70.0f, 50.0f);
    caps.initial_dhw_reduced_c  = sanitize_temp(caps.initial_dhw_reduced_c, 25.0f, 70.0f, 40.0f);
    caps.initial_dhw_tank_c     = sanitize_temp(caps.initial_dhw_tank_c, 0.0f, 95.0f, 45.0f);
    if (caps.initial_dhw_mode > static_cast<uint8_t>(DhwMode::Eco)) {
        caps.initial_dhw_mode = static_cast<uint8_t>(DhwMode::On);
    }
    sanitize_zone(caps.zone1);
    sanitize_zone(caps.zone2);
}

bool BsbDiscovery::load_from_nvs(BsbCapabilities& out_caps, bool allow_older_version) {
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    BsbCapabilities tmp = {};
    size_t required_size = sizeof(BsbCapabilities);
    err = nvs_get_blob(handle, NVS_KEY_CAPS, &tmp, &required_size);
    nvs_close(handle);

    if (err != ESP_OK || required_size != sizeof(BsbCapabilities)) {
        return false;
    }

    if (tmp.magic != CAPABILITIES_MAGIC) {
        return false;
    }
    // Une version antérieure de même format binaire n'est acceptée qu'à la demande (repli lorsque la
    // nouvelle découverte échoue) : ses critères de découverte sont périmés.
    const bool older_same_layout =
        tmp.version >= CAPABILITIES_OLDEST_SAME_LAYOUT && tmp.version < CAPABILITIES_VERSION;
    if (tmp.version != CAPABILITIES_VERSION && !(allow_older_version && older_same_layout)) {
        return false;
    }

    // Vérifier qu'au moins une fonctionnalité majeure est active
    if (!tmp.has_outdoor_temp && !tmp.has_hp_flow_temp && !tmp.zone1.present && !tmp.zone2.present) {
        return false;
    }

    sanitize_capabilities(tmp);
    out_caps = tmp;
    return true;
}

bool BsbDiscovery::save_to_nvs(const BsbCapabilities& caps) {
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erreur ouverture NVS: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_blob(handle, NVS_KEY_CAPS, &caps, sizeof(BsbCapabilities));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erreur sauvegarde NVS: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Topologie PAC sauvegardee en NVS avec succes.");
    return true;
}

void BsbDiscovery::request_rescan() {
    write_flag(NVS_KEY_RESCAN, 1);
}

bool BsbDiscovery::rescan_requested() {
    return read_flag(NVS_KEY_RESCAN) != 0;
}

void BsbDiscovery::clear_rescan_request() {
    write_flag(NVS_KEY_RESCAN, 0);
}

bool BsbDiscovery::external_sensor_used(uint8_t zone_index) {
    return read_flag(ext_used_key(zone_index)) != 0;
}

void BsbDiscovery::set_external_sensor_used(uint8_t zone_index, bool used) {
    write_flag(ext_used_key(zone_index), used ? 1 : 0);
}

// Une zone alimentée par une sonde externe répond sur 8740 / 8770 comme si elle avait une sonde filaire
static void forget_injected_room_sensor(ZoneCapabilities& zone, uint8_t zone_index) {
    if (zone.present && zone.has_room_sensor && BsbDiscovery::external_sensor_used(zone_index)) {
        ESP_LOGW(TAG, "Zone %u : la temperature d'ambiance relue est celle de la sonde externe injectee, "
                      "pas une sonde filaire.", zone_index);
        zone.has_room_sensor = false;
        zone.initial_room_c  = zone.initial_active_c;
    }
}

bool BsbDiscovery::clear_nvs() {
    nvs_handle_t handle = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    nvs_erase_key(handle, NVS_KEY_CAPS);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGW(TAG, "Topologie NVS effacee : une nouvelle decouverte sera executee.");
    return true;
}

bool BsbDiscovery::scan_heat_pump(BsbBus& bus, BsbCapabilities& out_caps) {
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "Demarrage de l'Auto-Decouverte de la Pompe a Chaleur BSB...");
    ESP_LOGI(TAG, "==========================================================");

    BsbCapabilities caps = {};
    caps.magic   = CAPABILITIES_MAGIC;
    caps.version = CAPABILITIES_VERSION;

    DecodedValue val;

    // 1. Identification du régulateur (jusqu'à 3 essais globaux si la PAC démarre)
    bool controller_reachable = false;
    for (int attempt = 0; attempt < 3 && !controller_reachable; ++attempt) {
        if (bus.query_sync(fields::CONTROLLER_MODEL, val, ValueType::String, 4, 400) && val.valid) {
            strncpy(caps.controller_model, val.str_val, sizeof(caps.controller_model) - 1);
            caps.controller_model[sizeof(caps.controller_model) - 1] = '\0';
            controller_reachable = true;
        } else if (bus.query_sync(fields::OUTDOOR_TEMP, val, ValueType::Temperature, 4, 400) && val.valid) {
            controller_reachable = true;
        } else {
            ESP_LOGW(TAG, "Attente reponse du regulateur BSB (tentative %d/3)...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (!controller_reachable) {
        ESP_LOGE(TAG, "Aucune reponse sur le bus BSB ! Verifiez le cablage RX(D7)/TX(D6) et l'alimentation de la PAC.");
        return false;
    }

    if (bus.query_sync(fields::SYSTEM_TYPE, val, ValueType::Int16, 3, 350) && val.valid) {
        caps.system_type = static_cast<int16_t>(val.int_val);
    }

    // 2. Capteurs généraux & Circuit Hydraulique PAC
    if (bus.query_sync(fields::OUTDOOR_TEMP, val, ValueType::Temperature, 4, 350) &&
        val.valid && val.float_val > -50.0f && val.float_val < 70.0f) {
        caps.has_outdoor_temp  = true;
        caps.initial_outdoor_c = val.float_val;
    }

    if (bus.query_sync(fields::HP_FLOW_TEMP, val, ValueType::Temperature, 4, 350) &&
        is_valid_flow_sensor(val)) {
        caps.has_hp_flow_temp  = true;
        caps.initial_hp_flow_c = val.float_val;
    }

    if (bus.query_sync(fields::HP_RETURN_TEMP, val, ValueType::Temperature, 4, 350) &&
        is_valid_flow_sensor(val)) {
        caps.has_hp_return_temp  = true;
        caps.initial_hp_return_c = val.float_val;
    }

    if (bus.query_sync(fields::COMPRESSOR_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
        caps.has_compressor_state = true;
    }

    if (bus.query_sync(fields::COMPRESSOR_MOD, val, ValueType::UInt8, 3, 350) && val.valid) {
        caps.has_compressor_mod = true;
    }

    // Les états tout-ou-rien d'un organe absent (pompe non câblée, sortie non affectée) reviennent
    // à « --- » : decode() les déclare alors invalides et l'organe n'est pas exposé.
    if (bus.query_sync(fields::HP_PUMP_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
        caps.has_hp_pump_state = true;
    }

    if (bus.query_sync(fields::BACKUP_HEATER_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
        caps.has_backup_heater = true;
    }

    // Modulation d'appoint : decode() ramène « --- » à 0 % (état normal d'un organe modulant à l'arrêt),
    // mais un appoint tout-ou-rien répond « --- » en permanence. Une résistance modulante étant rare, la
    // découverte exige une vraie valeur ; sinon l'entité resterait figée à 0 %.
    if (bus.query_sync(fields::BACKUP_MOD, val, ValueType::UInt8, 3, 350) && val.valid &&
        (val.flag_byte & 0x01) == 0) {
        caps.has_backup_mod = true;
    }

    if (bus.query_sync(fields::HP_STATUS_CODE, val, ValueType::UInt8, 3, 350) && val.valid) {
        caps.has_hp_status_code = true;
    }

    // 3. Découverte Zone 1 (Circuit Chauffage 1)
    if (bus.query_sync(fields::Z1_COMFORT_SETPOINT, val, ValueType::Temperature, 4, 350) &&
        is_valid_setpoint(val, 5.0f, 35.0f)) {
        caps.zone1.present           = true;
        caps.zone1.initial_comfort_c = val.float_val;

        if (bus.query_sync(fields::Z1_MODE, val, ValueType::UInt8, 3, 350) && val.valid && val.int_val <= 3) {
            caps.zone1.initial_mode = static_cast<uint8_t>(val.int_val);
        }
        if (bus.query_sync(fields::Z1_REDUCED_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 5.0f, 35.0f)) {
            caps.zone1.initial_reduced_c = val.float_val;
        }
        if (bus.query_sync(fields::Z1_ACTIVE_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 5.0f, 35.0f)) {
            caps.zone1.initial_active_c = val.float_val;
        } else {
            caps.zone1.initial_active_c = caps.zone1.initial_comfort_c;
        }

        // Sonde d'ambiance Zone 1 (filtre la valeur sentinelle 50.0°C si absente)
        if (bus.query_sync(fields::Z1_ROOM_TEMP, val, ValueType::Temperature, 3, 350) &&
            is_valid_room_sensor(val)) {
            caps.zone1.has_room_sensor = true;
            caps.zone1.initial_room_c  = val.float_val;
        } else {
            caps.zone1.has_room_sensor = false;
            caps.zone1.initial_room_c  = caps.zone1.initial_active_c;
        }

        // Sonde de départ Zone 1 (filtre la valeur sentinelle 0.0°C si absente)
        if (bus.query_sync(fields::Z1_FLOW_TEMP, val, ValueType::Temperature, 3, 350) &&
            is_valid_flow_sensor(val)) {
            caps.zone1.has_flow_sensor = true;
            caps.zone1.initial_flow_c  = val.float_val;
        }

        if (bus.query_sync(fields::Z1_PUMP_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
            caps.zone1.has_pump_state = true;
        }
        if (bus.query_sync(fields::Z1_STATUS_CODE, val, ValueType::UInt8, 3, 350) && val.valid) {
            caps.zone1.has_status_code = true;
        }
    }

    // 4. Découverte Zone 2 (Circuit Chauffage 2 / Extension AVS55)
    if (bus.query_sync(fields::Z2_COMFORT_SETPOINT, val, ValueType::Temperature, 4, 350) &&
        is_valid_setpoint(val, 5.0f, 35.0f)) {
        caps.zone2.present           = true;
        caps.zone2.initial_comfort_c = val.float_val;

        if (bus.query_sync(fields::Z2_MODE, val, ValueType::UInt8, 3, 350) && val.valid && val.int_val <= 3) {
            caps.zone2.initial_mode = static_cast<uint8_t>(val.int_val);
        }
        if (bus.query_sync(fields::Z2_REDUCED_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 5.0f, 35.0f)) {
            caps.zone2.initial_reduced_c = val.float_val;
        }
        if (bus.query_sync(fields::Z2_ACTIVE_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 5.0f, 35.0f)) {
            caps.zone2.initial_active_c = val.float_val;
        } else {
            caps.zone2.initial_active_c = caps.zone2.initial_comfort_c;
        }

        // Sonde d'ambiance Zone 2 (filtre 50.0°C)
        if (bus.query_sync(fields::Z2_ROOM_TEMP, val, ValueType::Temperature, 3, 350) &&
            is_valid_room_sensor(val)) {
            caps.zone2.has_room_sensor = true;
            caps.zone2.initial_room_c  = val.float_val;
        } else {
            caps.zone2.has_room_sensor = false;
            caps.zone2.initial_room_c  = caps.zone2.initial_active_c;
        }

        // Sonde de départ Zone 2 (filtre 0.0°C)
        if (bus.query_sync(fields::Z2_FLOW_TEMP, val, ValueType::Temperature, 3, 350) &&
            is_valid_flow_sensor(val)) {
            caps.zone2.has_flow_sensor = true;
            caps.zone2.initial_flow_c  = val.float_val;
        }

        if (bus.query_sync(fields::Z2_PUMP_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
            caps.zone2.has_pump_state = true;
        }
        if (bus.query_sync(fields::Z2_STATUS_CODE, val, ValueType::UInt8, 3, 350) && val.valid) {
            caps.zone2.has_status_code = true;
        }
    }

    // 5. Découverte Ballon Eau Chaude Sanitaire (ECS / DHW)
    if (bus.query_sync(fields::DHW_TANK_TEMP, val, ValueType::Temperature, 4, 350) &&
        is_valid_flow_sensor(val)) {
        caps.has_dhw           = true;
        caps.has_dhw_tank_temp = true;
        caps.initial_dhw_tank_c = val.float_val;

        if (bus.query_sync(fields::DHW_NOMINAL_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 25.0f, 70.0f)) {
            caps.initial_dhw_setpoint_c = val.float_val;
        }
        if (bus.query_sync(fields::DHW_REDUCED_SETPOINT, val, ValueType::Temperature, 3, 350) && val.valid) {
            // Le paramètre existe dès que le régulateur y répond avec une valeur exploitable ; la plage
            // ne sert qu'à retenir (ou non) cette valeur comme valeur initiale côté Zigbee.
            caps.has_dhw_reduced_setpoint = true;
            if (is_valid_setpoint(val, 25.0f, 70.0f)) {
                caps.initial_dhw_reduced_c = val.float_val;
            }
        }
        if (bus.query_sync(fields::DHW_MODE, val, ValueType::UInt8, 3, 350) && val.valid && val.int_val <= 2) {
            caps.initial_dhw_mode = static_cast<uint8_t>(val.int_val);
        }
        caps.has_dhw_status_code =
            bus.query_sync(fields::DHW_STATUS_CODE, val, ValueType::UInt8, 3, 350) && val.valid;
    }

    forget_injected_room_sensor(caps.zone1, 1);
    forget_injected_room_sensor(caps.zone2, 2);

    sanitize_capabilities(caps);
    out_caps = caps;
    log_capabilities(out_caps);
    return true;
}

void BsbDiscovery::refresh_initial_values(BsbBus& bus, BsbCapabilities& caps) {
    // Deux échecs consécutifs = régulateur injoignable : on garde l'instantané mémorisé plutôt que
    // de retarder le démarrage d'une attente par paramètre.
    uint8_t consecutive_failures = 0;
    uint8_t refreshed            = 0;

    auto read_temp = [&](uint32_t field, float min_c, float max_c, float& target) {
        if (consecutive_failures >= 2) {
            return;
        }
        DecodedValue val;
        if (!bus.query_sync(field, val, ValueType::Temperature, 2, 300)) {
            ++consecutive_failures;
            return;
        }
        consecutive_failures = 0;
        if (val.valid && val.float_val >= min_c && val.float_val <= max_c) {
            target = val.float_val;
            ++refreshed;
        }
    };
    auto read_mode = [&](uint32_t field, uint8_t max_value, uint8_t& target) {
        if (consecutive_failures >= 2) {
            return;
        }
        DecodedValue val;
        if (!bus.query_sync(field, val, ValueType::UInt8, 2, 300)) {
            ++consecutive_failures;
            return;
        }
        consecutive_failures = 0;
        if (val.valid && val.int_val >= 0 && val.int_val <= max_value) {
            target = static_cast<uint8_t>(val.int_val);
            ++refreshed;
        }
    };
    auto refresh_zone = [&](ZoneCapabilities& zone, uint32_t f_mode, uint32_t f_comfort, uint32_t f_reduced,
                            uint32_t f_active, uint32_t f_room, uint32_t f_flow) {
        if (!zone.present) {
            return;
        }
        read_mode(f_mode, static_cast<uint8_t>(HeatingMode::Comfort), zone.initial_mode);
        read_temp(f_comfort, 5.0f, 35.0f, zone.initial_comfort_c);
        read_temp(f_reduced, 5.0f, 35.0f, zone.initial_reduced_c);
        read_temp(f_active, 5.0f, 35.0f, zone.initial_active_c);
        if (zone.has_room_sensor) {
            read_temp(f_room, -5.0f, 44.0f, zone.initial_room_c);
        } else {
            zone.initial_room_c = zone.initial_active_c; // Même convention qu'à la découverte
        }
        if (zone.has_flow_sensor) {
            read_temp(f_flow, 4.0f, 95.0f, zone.initial_flow_c);
        }
    };

    if (caps.has_outdoor_temp)   read_temp(fields::OUTDOOR_TEMP, -50.0f, 70.0f, caps.initial_outdoor_c);
    if (caps.has_hp_flow_temp)   read_temp(fields::HP_FLOW_TEMP, 4.0f, 95.0f, caps.initial_hp_flow_c);
    if (caps.has_hp_return_temp) read_temp(fields::HP_RETURN_TEMP, 4.0f, 95.0f, caps.initial_hp_return_c);

    refresh_zone(caps.zone1, fields::Z1_MODE, fields::Z1_COMFORT_SETPOINT, fields::Z1_REDUCED_SETPOINT,
                 fields::Z1_ACTIVE_SETPOINT, fields::Z1_ROOM_TEMP, fields::Z1_FLOW_TEMP);
    refresh_zone(caps.zone2, fields::Z2_MODE, fields::Z2_COMFORT_SETPOINT, fields::Z2_REDUCED_SETPOINT,
                 fields::Z2_ACTIVE_SETPOINT, fields::Z2_ROOM_TEMP, fields::Z2_FLOW_TEMP);

    if (caps.has_dhw) {
        read_mode(fields::DHW_MODE, static_cast<uint8_t>(DhwMode::Eco), caps.initial_dhw_mode);
        read_temp(fields::DHW_NOMINAL_SETPOINT, 25.0f, 70.0f, caps.initial_dhw_setpoint_c);
        if (caps.has_dhw_reduced_setpoint) {
            read_temp(fields::DHW_REDUCED_SETPOINT, 25.0f, 70.0f, caps.initial_dhw_reduced_c);
        }
        if (caps.has_dhw_tank_temp) {
            read_temp(fields::DHW_TANK_TEMP, 4.0f, 95.0f, caps.initial_dhw_tank_c);
        }
    }

    sanitize_capabilities(caps);
    if (consecutive_failures >= 2) {
        ESP_LOGW(TAG, "Regulateur injoignable au demarrage : valeurs initiales memorisees conservees (%u relue(s)).",
                 refreshed);
    } else {
        ESP_LOGI(TAG, "Valeurs initiales relues sur le bus BSB : %u (consignes, modes, temperatures).", refreshed);
    }
}

void BsbDiscovery::log_capabilities(const BsbCapabilities& caps) {
    ESP_LOGI(TAG, "---------------- BILAN AUTO-DECOUVERTE PAC ----------------");
    ESP_LOGI(TAG, "  Modele Regulateur : %s (Systeme type %d)", caps.controller_model, caps.system_type);
    ESP_LOGI(TAG, "  Sonde Exterieure  : %s (%.1f C)", caps.has_outdoor_temp ? "OUI" : "NON", caps.initial_outdoor_c);
    ESP_LOGI(TAG, "  Hydraulique PAC   : Depart=%s (%.1f C), Retour=%s (%.1f C)",
             caps.has_hp_flow_temp ? "OUI" : "NON", caps.initial_hp_flow_c,
             caps.has_hp_return_temp ? "OUI" : "NON", caps.initial_hp_return_c);
    ESP_LOGI(TAG, "  Compresseur       : Etat=%s, Modulation=%s",
             caps.has_compressor_state ? "OUI" : "NON",
             caps.has_compressor_mod ? "OUI" : "NON");
    ESP_LOGI(TAG, "  Pompe PAC (8406)  : %s",
             caps.has_hp_pump_state ? "OUI" : "NON");
    ESP_LOGI(TAG, "  Appoint Electrique: Etat=%s, Modulation=%s",
             caps.has_backup_heater ? "OUI" : "NON",
             caps.has_backup_mod ? "OUI" : "NON");
    ESP_LOGI(TAG, "  Zone 1 (Circuit 1): %s | Sonde Ambiance=%s | Sonde Depart=%s | Pompe=%s",
             caps.zone1.present ? "PRESENTE" : "ABSENTE",
             caps.zone1.has_room_sensor ? "OUI" : "NON (filtree)",
             caps.zone1.has_flow_sensor ? "OUI" : "NON (filtree)",
             caps.zone1.has_pump_state ? "OUI" : "NON");
    ESP_LOGI(TAG, "  Zone 2 (Circuit 2): %s | Sonde Ambiance=%s | Sonde Depart=%s | Pompe=%s",
             caps.zone2.present ? "PRESENTE" : "ABSENTE",
             caps.zone2.has_room_sensor ? "OUI" : "NON (filtree)",
             caps.zone2.has_flow_sensor ? "OUI" : "NON (filtree)",
             caps.zone2.has_pump_state ? "OUI" : "NON");
    ESP_LOGI(TAG, "  Ballon ECS (DHW)  : %s (Ballon=%.1f C, Nominale=%.1f C, Reduite=%.1f C, Mode=%u)",
             caps.has_dhw ? "PRESENT" : "ABSENT",
             caps.initial_dhw_tank_c, caps.initial_dhw_setpoint_c, caps.initial_dhw_reduced_c, caps.initial_dhw_mode);
    ESP_LOGI(TAG, "-----------------------------------------------------------");
}

void BsbDiscovery::register_discovered_polls(BsbBus& bus, const BsbCapabilities& caps) {
    bus.clear_poll_fields();

    // L'ordre d'enregistrement fixe l'ordre du premier tour de lecture (au démarrage, puis après chaque
    // resynchronisation, les échéances sont échelonnées dans cet ordre). On commence donc par ce qui
    // détermine l'état de chauffe affiché par les thermostats ; consignes, modes et températures ont
    // déjà été relus au démarrage et peuvent attendre la fin du tour.

    // 1. Générateur et codes d'état : état de chauffe des trois thermostats
    if (caps.has_compressor_state) bus.register_poll_field(fields::COMPRESSOR_STATE,    15000);
    if (caps.has_compressor_mod)   bus.register_poll_field(fields::COMPRESSOR_MOD,      15000);
    if (caps.has_hp_status_code)   bus.register_poll_field(fields::HP_STATUS_CODE,      15000);
    if (caps.zone1.present && caps.zone1.has_status_code) bus.register_poll_field(fields::Z1_STATUS_CODE, 15000);
    if (caps.zone2.present && caps.zone2.has_status_code) bus.register_poll_field(fields::Z2_STATUS_CODE, 15000);
    // Le code statut ECS (8003) pilote l'état de chauffe du thermostat ballon : toujours scruté
    if (caps.has_dhw)              bus.register_poll_field(fields::DHW_STATUS_CODE,     15000);
    if (caps.has_backup_heater)    bus.register_poll_field(fields::BACKUP_HEATER_STATE, 30000);
    if (caps.has_backup_mod)       bus.register_poll_field(fields::BACKUP_MOD,          30000);
    // Départ et retour PAC : l'écart entre les deux confirme que la chaleur circule réellement
    if (caps.has_hp_flow_temp)     bus.register_poll_field(fields::HP_FLOW_TEMP,        20000);
    if (caps.has_hp_return_temp)   bus.register_poll_field(fields::HP_RETURN_TEMP,      20000);

    // 2. Pompes
    if (caps.has_hp_pump_state)    bus.register_poll_field(fields::HP_PUMP_STATE,       15000);
    if (caps.zone1.present && caps.zone1.has_pump_state) bus.register_poll_field(fields::Z1_PUMP_STATE, 25000);
    if (caps.zone2.present && caps.zone2.has_pump_state) bus.register_poll_field(fields::Z2_PUMP_STATE, 25000);

    // 3. Modes
    if (caps.zone1.present) bus.register_poll_field(fields::Z1_MODE,  45000);
    if (caps.zone2.present) bus.register_poll_field(fields::Z2_MODE,  45000);
    if (caps.has_dhw)       bus.register_poll_field(fields::DHW_MODE, 60000);

    // 4. Températures
    if (caps.has_outdoor_temp) bus.register_poll_field(fields::OUTDOOR_TEMP, 20000);
    if (caps.zone1.present) {
        // Sans sonde filaire, 8740 est tout de même relu (lentement) : c'est lui qui prouve que le
        // régulateur prend bien en compte la température d'ambiance externe injectée par INF.
        bus.register_poll_field(fields::Z1_ROOM_TEMP, caps.zone1.has_room_sensor ? 20000 : 60000);
        if (caps.zone1.has_flow_sensor) bus.register_poll_field(fields::Z1_FLOW_TEMP, 25000);
    }
    if (caps.zone2.present) {
        bus.register_poll_field(fields::Z2_ROOM_TEMP, caps.zone2.has_room_sensor ? 15000 : 60000);
        if (caps.zone2.has_flow_sensor) bus.register_poll_field(fields::Z2_FLOW_TEMP, 25000);
    }
    if (caps.has_dhw && caps.has_dhw_tank_temp) bus.register_poll_field(fields::DHW_TANK_TEMP, 30000);

    // 5. Consignes (elles ne changent que sur action de l'utilisateur)
    if (caps.zone1.present) {
        bus.register_poll_field(fields::Z1_ACTIVE_SETPOINT,  45000);
        bus.register_poll_field(fields::Z1_COMFORT_SETPOINT, 60000);
        bus.register_poll_field(fields::Z1_REDUCED_SETPOINT, 90000);
    }
    if (caps.zone2.present) {
        bus.register_poll_field(fields::Z2_ACTIVE_SETPOINT,  45000);
        bus.register_poll_field(fields::Z2_COMFORT_SETPOINT, 60000);
        bus.register_poll_field(fields::Z2_REDUCED_SETPOINT, 90000);
    }
    if (caps.has_dhw) {
        bus.register_poll_field(fields::DHW_NOMINAL_SETPOINT, 90000);
        if (caps.has_dhw_reduced_setpoint) bus.register_poll_field(fields::DHW_REDUCED_SETPOINT, 120000);
    }
}

} // namespace bsb

#endif // defined(ESP_PLATFORM)

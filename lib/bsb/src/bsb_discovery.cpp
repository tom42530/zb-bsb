#if defined(ESP_PLATFORM)

#include "bsb_discovery.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstring>

namespace bsb {

static const char* TAG           = "bsb.discovery";
static const char* NVS_NAMESPACE = "zb_bsb";
static const char* NVS_KEY_CAPS  = "caps_v1";

bool BsbDiscovery::load_from_nvs(BsbCapabilities& out_caps) {
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

    if (tmp.magic != CAPABILITIES_MAGIC || tmp.version != CAPABILITIES_VERSION) {
        return false;
    }

    // Vérifier qu'au moins une fonctionnalité majeure est active
    if (!tmp.has_outdoor_temp && !tmp.has_hp_flow_temp && !tmp.zone1.present && !tmp.zone2.present) {
        return false;
    }

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

    if (bus.query_sync(fields::HP_PUMP_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
        caps.has_hp_pump_state = true;
    }

    if (bus.query_sync(fields::BACKUP_HEATER_STATE, val, ValueType::Binary, 3, 350) && val.valid) {
        caps.has_backup_heater = true;
    }

    if (bus.query_sync(fields::BACKUP_MOD, val, ValueType::UInt8, 3, 350) && val.valid) {
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
        if (bus.query_sync(fields::DHW_REDUCED_SETPOINT, val, ValueType::Temperature, 3, 350) &&
            is_valid_setpoint(val, 25.0f, 70.0f)) {
            caps.has_dhw_reduced_setpoint = true;
            caps.initial_dhw_reduced_c    = val.float_val;
        }
        if (bus.query_sync(fields::DHW_MODE, val, ValueType::UInt8, 3, 350) && val.valid && val.int_val <= 2) {
            caps.initial_dhw_mode = static_cast<uint8_t>(val.int_val);
        }
        caps.has_dhw_status_code = true;
        if (bus.query_sync(fields::DHW_STATUS_CODE, val, ValueType::UInt8, 3, 350) && val.valid) {
            // Statut ECS initialisé
        }
    }

    out_caps = caps;
    log_capabilities(out_caps);
    return true;
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

    // Capteurs généraux PAC
    if (caps.has_outdoor_temp)     bus.register_poll_field(fields::OUTDOOR_TEMP,        20000);
    if (caps.has_hp_flow_temp)     bus.register_poll_field(fields::HP_FLOW_TEMP,        20000);
    if (caps.has_hp_return_temp)   bus.register_poll_field(fields::HP_RETURN_TEMP,      20000);
    if (caps.has_compressor_state) bus.register_poll_field(fields::COMPRESSOR_STATE,    20000);
    if (caps.has_compressor_mod)   bus.register_poll_field(fields::COMPRESSOR_MOD,      20000);
    if (caps.has_hp_pump_state)    bus.register_poll_field(fields::HP_PUMP_STATE,       20000);
    if (caps.has_backup_heater)    bus.register_poll_field(fields::BACKUP_HEATER_STATE, 30000);
    if (caps.has_backup_mod)       bus.register_poll_field(fields::BACKUP_MOD,          30000);
    if (caps.has_hp_status_code)   bus.register_poll_field(fields::HP_STATUS_CODE,      45000);

    // Zone 1
    if (caps.zone1.present) {
        bus.register_poll_field(fields::Z1_MODE,             45000);
        bus.register_poll_field(fields::Z1_COMFORT_SETPOINT, 60000);
        bus.register_poll_field(fields::Z1_REDUCED_SETPOINT, 90000);
        bus.register_poll_field(fields::Z1_ACTIVE_SETPOINT,  45000);
        if (caps.zone1.has_room_sensor) bus.register_poll_field(fields::Z1_ROOM_TEMP,  20000);
        if (caps.zone1.has_flow_sensor) bus.register_poll_field(fields::Z1_FLOW_TEMP,  25000);
        if (caps.zone1.has_pump_state && !caps.has_hp_pump_state) {
            bus.register_poll_field(fields::Z1_PUMP_STATE, 25000);
        }
        if (caps.zone1.has_status_code) bus.register_poll_field(fields::Z1_STATUS_CODE,45000);
    }

    // Zone 2
    if (caps.zone2.present) {
        bus.register_poll_field(fields::Z2_MODE,             45000);
        bus.register_poll_field(fields::Z2_COMFORT_SETPOINT, 60000);
        bus.register_poll_field(fields::Z2_REDUCED_SETPOINT, 90000);
        bus.register_poll_field(fields::Z2_ACTIVE_SETPOINT,  45000);
        if (caps.zone2.has_room_sensor) bus.register_poll_field(fields::Z2_ROOM_TEMP,  15000);
        if (caps.zone2.has_flow_sensor) bus.register_poll_field(fields::Z2_FLOW_TEMP,  25000);
        if (caps.zone2.has_pump_state)  bus.register_poll_field(fields::Z2_PUMP_STATE, 25000);
        if (caps.zone2.has_status_code) bus.register_poll_field(fields::Z2_STATUS_CODE,45000);
    }

    // Ballon ECS (DHW)
    if (caps.has_dhw) {
        bus.register_poll_field(fields::DHW_MODE,             60000);
        bus.register_poll_field(fields::DHW_NOMINAL_SETPOINT, 90000);
        if (caps.has_dhw_reduced_setpoint) bus.register_poll_field(fields::DHW_REDUCED_SETPOINT, 120000);
        if (caps.has_dhw_tank_temp)   bus.register_poll_field(fields::DHW_TANK_TEMP,   30000);
        bus.register_poll_field(fields::DHW_STATUS_CODE, 45000);
    }
}

} // namespace bsb

#endif // defined(ESP_PLATFORM)

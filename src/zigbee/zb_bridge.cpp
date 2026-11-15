#include "zb_bridge.h"
#include <Arduino.h>
#include "board_config.h"
#include "bsb_discovery.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_ota.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "zcl/esp_zigbee_zcl_binary_input.h"
#include "zcl/esp_zigbee_zcl_analog_input.h"
#include "zcl/esp_zigbee_zcl_analog_output.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace zb_bsb {

static const char* TAG = "zb.bridge";

// Unités d'ingénierie BACnet/ZCL standard (attribut 0x0075 EngineeringUnits)
static constexpr uint16_t ZCL_UNIT_CELSIUS       = 0x003E; // 62 = Degrees Celsius (°C)
static constexpr uint16_t ZCL_UNIT_PERCENT       = 0x0062; // 98 = Percent (%)
static constexpr uint16_t ZCL_UNIT_DIMENSIONLESS = 0x005F; // 95 = No units (Code statut)

// Cadence de ré-émission BSB INF pour maintenir la sonde d'ambiance externe active sur RVS21
static constexpr uint64_t EXT_SENSOR_BSB_REBROADCAST_MS = 60000ULL;   // 60 secondes
// Watchdog de sécurité : retour sur sonde interne BSB si aucune mise à jour depuis 30 minutes
static constexpr uint64_t EXT_SENSOR_WATCHDOG_TIMEOUT_MS = 1800000ULL; // 30 minutes
// Sonde liée par Binding Zigbee direct : un thermomètre sur pile ne rapporte souvent qu'une fois par heure
// lorsque la pièce est stable. Le watchdog doit donc être plus long que cet intervalle, sinon la sonde
// externe serait désactivée puis réactivée à chaque rapport.
static constexpr uint64_t EXT_SENSOR_BINDING_WATCHDOG_MS = 10800000ULL; // 3 heures
// Délai au-delà duquel on signale que le régulateur ne relit pas la température injectée (8740 / 8770)
static constexpr uint64_t EXT_SENSOR_CONFIRM_TIMEOUT_MS = 300000ULL;   // 5 minutes
// Plage acceptée pour une température d'ambiance externe (en dessous : désactivation, au-dessus : ignorée)
static constexpr float EXT_SENSOR_MIN_C = 4.0f;
static constexpr float EXT_SENSOR_MAX_C = 38.0f;

// Plages de consignes acceptées depuis Zigbee (identiques aux limites Min/Max exposées par les thermostats)
static constexpr float ZONE_SETPOINT_MIN_C = 7.0f;
static constexpr float ZONE_SETPOINT_MAX_C = 30.0f;
static constexpr float DHW_SETPOINT_MIN_C  = 30.0f;
static constexpr float DHW_SETPOINT_MAX_C  = 65.0f;

// Silence du bus BSB au-delà duquel l'état mémorisé de la PAC n'est plus considéré comme fiable
static constexpr uint64_t BSB_STALE_TIMEOUT_MS = 180000ULL; // 3 minutes

static uint64_t now_ms() {
    return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
}

// Durée écoulée depuis un horodatage, saturée à 0. Un horodatage peut être posé par l'autre tâche
// juste APRÈS notre lecture de l'horloge (pendant l'attente du verrou Zigbee) : la soustraction non
// signée brute produirait alors une durée gigantesque et déclencherait à tort les temporisations.
static uint64_t elapsed_ms(uint64_t now, uint64_t since) {
    return (now > since) ? (now - since) : 0;
}

// ----------------------------------------------------------------------------
// État du transfert OTA en cours (toujours manipulé sous verrou Zigbee)
// ----------------------------------------------------------------------------
static constexpr uint16_t OTA_ELEMENT_HEADER_LEN = 6; // [uint16_t tag_id, uint32_t length]
static constexpr uint16_t OTA_UPGRADE_IMAGE_TAG  = 0x0000;
static constexpr uint64_t OTA_STALL_TIMEOUT_MS   = 600000ULL; // 10 minutes sans aucun bloc reçu

static const esp_partition_t* s_ota_partition        = nullptr;
static esp_ota_handle_t       s_ota_handle           = 0;
static bool                   s_ota_tag_received     = false;
static uint16_t               s_ota_tag_id           = 0xFFFF;
static uint32_t               s_ota_total_size       = 0;
static uint32_t               s_ota_offset           = 0;
static int64_t                s_ota_start_time_us    = 0;
static uint64_t               s_ota_last_activity_ms = 0;
static uint8_t                s_ota_last_pct_log     = 255;

static void ota_reset_transfer_state() {
    s_ota_offset       = 0;
    s_ota_total_size   = 0;
    s_ota_tag_received = false;
    s_ota_tag_id       = 0xFFFF;
}

static void ota_abort_transfer() {
    if (s_ota_handle) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }
    ota_reset_transfer_state();
}

// Formatage chaîne ZCL (1er octet = longueur)
static void make_zcl_string(char* dst, size_t dst_size, const char* text) {
    if (!dst || dst_size < 2) return;
    size_t len = text ? strlen(text) : 0;
    if (len > dst_size - 2) {
        len = dst_size - 2;
    }
    dst[0] = static_cast<char>(len);
    if (len > 0) {
        memcpy(dst + 1, text, len);
    }
    dst[len + 1] = '\0';
}

static int16_t temp_to_zcl_s16(float temp_c) {
    if (!std::isfinite(temp_c)) {
        return INT16_MIN; // 0x8000 = « valeur invalide » en ZCL
    }
    // Borné à la plage représentable (centièmes de degré) : une valeur aberrante ne doit pas déborder
    const float centi = temp_c * 100.0f;
    if (centi <= -27315.0f) return -27315;
    if (centi >= 32767.0f) return 32767;
    return static_cast<int16_t>(std::lround(centi));
}

static float zcl_s16_to_temp(int16_t val) {
    return static_cast<float>(val) / 100.0f;
}

ZbBridge& ZbBridge::instance() {
    static ZbBridge inst;
    return inst;
}

// Chacun des quatre régimes du régulateur a sa valeur SystemMode. C'est le contrat sur lequel reposent
// le quirk ZHA et le convertisseur Zigbee2MQTT pour présenter modes et préréglages :
//   Hors-gel = Off (0x00), Automatique = Auto (0x01), Réduit / Éco = Sleep (0x09), Confort = Heat (0x04).
uint8_t ZbBridge::bsb_mode_to_zcl_system_mode(uint8_t bsb_mode) {
    switch (static_cast<bsb::HeatingMode>(bsb_mode)) {
        case bsb::HeatingMode::Protection:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   // 0x00 = Hors-gel
        case bsb::HeatingMode::Auto:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO;  // 0x01 = Programme horaire du régulateur
        case bsb::HeatingMode::Reduced:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP; // 0x09 = Réduit permanent (Éco)
        case bsb::HeatingMode::Comfort:
        default:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;  // 0x04 = Confort permanent
    }
}

bool ZbBridge::zcl_system_mode_to_bsb_mode(uint8_t zcl_mode, uint8_t& bsb_mode) {
    switch (zcl_mode) {
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF:
            bsb_mode = static_cast<uint8_t>(bsb::HeatingMode::Protection);
            return true;
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO:
            bsb_mode = static_cast<uint8_t>(bsb::HeatingMode::Auto);
            return true;
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP:
            bsb_mode = static_cast<uint8_t>(bsb::HeatingMode::Reduced);
            return true;
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT:
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_EMERGENCY_HEATING:
            bsb_mode = static_cast<uint8_t>(bsb::HeatingMode::Comfort);
            return true;
        default:
            // Rafraîchissement, ventilation, déshumidification... : sans équivalent sur un circuit de chauffage
            return false;
    }
}

// Mode ECS (Prog 1600) -> SystemMode du thermostat ballon, selon la même convention que les zones :
// Arrêt = Off, Éco = Sleep, Marche = Heat.
static uint8_t dhw_mode_to_zcl_system_mode(uint8_t dhw_mode) {
    switch (static_cast<bsb::DhwMode>(dhw_mode)) {
        case bsb::DhwMode::Off:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
        case bsb::DhwMode::Eco:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP;
        case bsb::DhwMode::On:
        default:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;
    }
}

// ============================================================================
// Helpers de création de clusters ZCL enrichis (compatibilité native ZHA & Z2M)
// ============================================================================

static void add_basic_and_identify_clusters(esp_zb_cluster_list_t* cl, const char* sw_build_id, bool is_primary_ep) {
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01, // Mains (single phase)
    };
    esp_zb_attribute_list_t* basic_attr = esp_zb_basic_cluster_create(&basic_cfg);

    if (is_primary_ep) {
        char manuf[32];
        char model[32];
        char sw_id[32];
        make_zcl_string(manuf, sizeof(manuf), "Atlantic-Siemens");
        make_zcl_string(model, sizeof(model), "ZB-BSB");
        make_zcl_string(sw_id, sizeof(sw_id), (sw_build_id && sw_build_id[0]) ? sw_build_id : "RVS21");

        uint8_t app_version = static_cast<uint8_t>((ZB_BSB_OTA_FILE_VERSION >> 24) & 0xFF);
        uint8_t hw_version  = static_cast<uint8_t>(ZB_BSB_OTA_HW_VERSION & 0xFF);

        esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manuf);
        esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
        esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, sw_id);
        esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
        esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);
    }

    esp_zb_cluster_list_add_basic_cluster(cl, basic_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t id_cfg = {.identify_time = 0};
    esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(&id_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void add_ota_client_cluster(esp_zb_cluster_list_t* cl) {
    esp_zb_ota_cluster_cfg_t ota_cfg = {};
    ota_cfg.ota_upgrade_file_version        = ZB_BSB_OTA_FILE_VERSION;
    ota_cfg.ota_upgrade_downloaded_file_ver = ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE;
    ota_cfg.ota_upgrade_manufacturer        = ZB_BSB_OTA_MANUFACTURER_CODE;
    ota_cfg.ota_upgrade_image_type          = ZB_BSB_OTA_IMAGE_TYPE;

    esp_zb_attribute_list_t* ota_attr = esp_zb_ota_cluster_create(&ota_cfg);

    esp_zb_zcl_ota_upgrade_client_variable_t variable_cfg = {};
    variable_cfg.timer_query   = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF;
    variable_cfg.hw_version    = ZB_BSB_OTA_HW_VERSION;
    variable_cfg.max_data_size = ZB_BSB_OTA_MAX_DATA_SIZE;

    uint16_t ota_server_addr = 0x0000; // Coordinateur par défaut (supporte immédiatement zha_toolkit.ota_notify)
    uint8_t  ota_server_ep   = 1;

    esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &variable_cfg);
    esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &ota_server_addr);
    esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &ota_server_ep);

    esp_zb_cluster_list_add_ota_cluster(cl, ota_attr, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
}

static void add_temp_measurement_cluster(esp_zb_cluster_list_t* cl, float initial_c, float min_c, float max_c) {
    esp_zb_temperature_meas_cluster_cfg_t cfg = {
        .measured_value = temp_to_zcl_s16(initial_c),
        .min_value      = temp_to_zcl_s16(min_c),
        .max_value      = temp_to_zcl_s16(max_c),
    };
    esp_zb_attribute_list_t* attr = esp_zb_temperature_meas_cluster_create(&cfg);
    esp_zb_cluster_list_add_temperature_meas_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void add_binary_input_cluster(esp_zb_cluster_list_t* cl, const char* description, bool initial_val) {
    esp_zb_binary_input_cluster_cfg_t cfg = {
        .out_of_service = false,
        .status_flags   = 0,
        .present_value  = initial_val,
    };
    esp_zb_attribute_list_t* attr = esp_zb_binary_input_cluster_create(&cfg);

    char zcl_desc[40];
    make_zcl_string(zcl_desc, sizeof(zcl_desc), description);
    uint32_t app_type = (0x03UL << 24);

    esp_zb_binary_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, zcl_desc);
    esp_zb_binary_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_BINARY_INPUT_APPLICATION_TYPE_ID, &app_type);

    esp_zb_cluster_list_add_binary_input_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void add_analog_input_cluster(esp_zb_cluster_list_t* cl,
                                     const char* description,
                                     float initial_val,
                                     float min_val,
                                     float max_val,
                                     float resolution,
                                     uint16_t engineering_units) {
    esp_zb_analog_input_cluster_cfg_t cfg = {
        .out_of_service = false,
        .present_value  = initial_val,
        .status_flags   = 0,
    };
    esp_zb_attribute_list_t* attr = esp_zb_analog_input_cluster_create(&cfg);

    char zcl_desc[40];
    make_zcl_string(zcl_desc, sizeof(zcl_desc), description);
    uint16_t units = engineering_units;

    esp_zb_analog_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, zcl_desc);
    esp_zb_analog_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, &min_val);
    esp_zb_analog_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, &max_val);
    esp_zb_analog_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID, &resolution);
    esp_zb_analog_input_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units);

    esp_zb_cluster_list_add_analog_input_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void add_analog_output_cluster(esp_zb_cluster_list_t* cl,
                                      const char* description,
                                      float initial_val,
                                      float min_val,
                                      float max_val,
                                      float resolution,
                                      uint16_t engineering_units) {
    esp_zb_analog_output_cluster_cfg_t cfg = {
        .out_of_service = false,
        .present_value  = initial_val,
        .status_flags   = 0,
    };
    esp_zb_attribute_list_t* attr = esp_zb_analog_output_cluster_create(&cfg);

    char zcl_desc[40];
    make_zcl_string(zcl_desc, sizeof(zcl_desc), description);
    uint16_t units = engineering_units;
    uint32_t app_type = (ESP_ZB_ZCL_AO_GROUP_ID << 24) | ESP_ZB_ZCL_AO_TEMPERATURE_ZONE_TEMPERATURE_SETPOINT;

    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID, zcl_desc);
    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &min_val);
    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &max_val);
    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID, &resolution);
    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, &units);
    esp_zb_analog_output_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_APPLICATION_TYPE_ID, &app_type);

    esp_zb_cluster_list_add_analog_output_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void enable_attribute_reporting(esp_zb_attribute_list_t* attr_list, uint16_t attr_id) {
    for (esp_zb_attribute_list_t* a = attr_list; a != nullptr; a = a->next) {
        if (a->attribute.id == attr_id) {
            a->attribute.access |= ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
            break;
        }
    }
}

static void add_thermostat_cluster(esp_zb_cluster_list_t* cl,
                                   const bsb::ZoneCapabilities& zcaps,
                                   float outdoor_c) {
    esp_zb_thermostat_cluster_cfg_t cfg = {
        .local_temperature             = temp_to_zcl_s16(zcaps.initial_room_c),
        .occupied_cooling_setpoint     = 5000, // 50.0 °C (évite le conflit ZBOSS de bande morte chaud/froid sur consigne max 30 °C)
        .occupied_heating_setpoint     = temp_to_zcl_s16(zcaps.initial_comfort_c),
        .control_sequence_of_operation = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY,
        .system_mode                   = ZbBridge::bsb_mode_to_zcl_system_mode(zcaps.initial_mode),
    };

    esp_zb_attribute_list_t* attr = esp_zb_thermostat_cluster_create(&cfg);

    int16_t  unoccupied_heat = temp_to_zcl_s16(zcaps.initial_reduced_c);
    int16_t  abs_min_heat    = 700;  // 7.0 °C (limite minimale pour ZHA)
    int16_t  abs_max_heat    = 3000; // 30.0 °C (limite maximale pour ZHA)
    int16_t  outdoor_s16     = temp_to_zcl_s16(outdoor_c);
    uint8_t  remote_sensing  = 0x00; // Bit 0 = 1 lorsque la sonde externe Home Assistant est active
    uint16_t running_state   = 0x0000;
    uint8_t  pi_demand       = 0;

    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, &unoccupied_heat);
    // On utilise les attributs AbsMin/Max et Min/Max pour borner de façon stricte les consignes 7-30 °C dans ZHA
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, &outdoor_s16);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, &remote_sensing);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &running_state);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, &pi_demand);

    // Active l'accès REPORTING dans ZBOSS pour permettre les rapports ZCL instantanés vers Home Assistant
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID);

    esp_zb_cluster_list_add_thermostat_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Ajout du cluster TemperatureMeasurement en CLIENT_ROLE sur l'Endpoint du Thermostat
    // pour supporter également le Binding Zigbee direct depuis n'importe quel thermomètre Zigbee !
    esp_zb_cluster_list_add_temperature_meas_cluster(cl, esp_zb_temperature_meas_cluster_create(nullptr), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
}

static void add_dhw_thermostat_cluster(esp_zb_cluster_list_t* cl,
                                       float tank_c,
                                       float nominal_setpoint_c,
                                       float reduced_setpoint_c,
                                       uint8_t dhw_mode) {
    uint8_t zcl_sys_mode = dhw_mode_to_zcl_system_mode(dhw_mode);

    esp_zb_thermostat_cluster_cfg_t cfg = {
        .local_temperature             = temp_to_zcl_s16(tank_c),
        .occupied_cooling_setpoint     = 9500, // 95.0 °C (requis par ZBOSS pour autoriser une consigne de chauffe ECS jusqu'à 65.0 °C sans conflit de consigne froide/morte)
        .occupied_heating_setpoint     = temp_to_zcl_s16(nominal_setpoint_c),
        .control_sequence_of_operation = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY,
        .system_mode                   = zcl_sys_mode,
    };

    esp_zb_attribute_list_t* attr = esp_zb_thermostat_cluster_create(&cfg);

    int16_t  unoccupied_heat = temp_to_zcl_s16(reduced_setpoint_c);
    int16_t  abs_min_heat    = 3000; // 30.0 °C
    int16_t  abs_max_heat    = 6500; // 65.0 °C
    uint16_t running_state   = 0x0000;
    uint8_t  pi_demand       = 0;

    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, &unoccupied_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &running_state);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, &pi_demand);

    // Active l'accès REPORTING dans ZBOSS pour permettre les rapports ZCL instantanés vers Home Assistant
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID);
    enable_attribute_reporting(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID);

    esp_zb_cluster_list_add_thermostat_cluster(cl, attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

// ============================================================================
// Construction dynamique des Endpoints selon la topologie découverte
// ============================================================================

void ZbBridge::build_endpoints(esp_zb_ep_list_t* ep_list) {
    // ------------------------------------------------------------------------
    // ENDPOINT 1 : Maître PAC + Sonde Extérieure + Compresseur + Thermostat Zone 1
    // ------------------------------------------------------------------------
    {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, true);
        add_ota_client_cluster(cl);

        if (caps_.zone1.present) {
            add_thermostat_cluster(cl, caps_.zone1, caps_.initial_outdoor_c);
            add_analog_output_cluster(cl, "Consigne Reduite Zone 1",
                                      caps_.zone1.initial_reduced_c, 7.0f, 30.0f, 0.5f, ZCL_UNIT_CELSIUS);
        }
        if (caps_.has_outdoor_temp) {
            add_temp_measurement_cluster(cl, caps_.initial_outdoor_c, -40.0f, 60.0f);
        }
        if (caps_.has_compressor_state) {
            add_binary_input_cluster(cl, "Compresseur PAC", false);
        }
        if (caps_.has_compressor_mod) {
            add_analog_input_cluster(cl, "Modulation Compresseur", 0.0f, 0.0f, 100.0f, 1.0f, ZCL_UNIT_PERCENT);
        }

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_MASTER_Z1,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Maitre PAC + Thermostat Zone 1)", EP_MASTER_Z1);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 2 : Zone 2 (Thermostat + Ambiance Z2 + Pompe Q6 + Statut Z2)
    //              Uniquement enregistré si la Zone 2 existe sur la PAC !
    // ------------------------------------------------------------------------
    if (caps_.zone2.present) {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        // INDEX 2 : Thermostat Zone 2 (identique à l'index 2 de l'EP1 pour ZBOSS)
        add_thermostat_cluster(cl, caps_.zone2, caps_.initial_outdoor_c);
        add_analog_output_cluster(cl, "Consigne Reduite Zone 2",
                                  caps_.zone2.initial_reduced_c, 7.0f, 30.0f, 0.5f, ZCL_UNIT_CELSIUS);

        if (caps_.zone2.has_room_sensor) {
            add_temp_measurement_cluster(cl, caps_.zone2.initial_room_c, 0.0f, 45.0f);
        } else if (caps_.zone2.has_flow_sensor) {
            add_temp_measurement_cluster(cl, caps_.zone2.initial_flow_c, 5.0f, 80.0f);
        }

        if (caps_.zone2.has_pump_state) {
            add_binary_input_cluster(cl, "Pompe Zone 2 (Q6)", false);
        }
        if (caps_.zone2.has_status_code) {
            add_analog_input_cluster(cl, "Code Statut Zone 2", 0.0f, 0.0f, 255.0f, 1.0f, ZCL_UNIT_DIMENSIONLESS);
        }

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_ZONE2,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Thermostat & Capteurs Zone 2)", EP_ZONE2);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 3 : Départ PAC + Pompe Z1 + Statut PAC + Sonde Temp. Externe Zone 1
    // ------------------------------------------------------------------------
    if (caps_.has_hp_flow_temp || caps_.zone1.present || caps_.has_hp_status_code) {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        if (caps_.has_hp_flow_temp) {
            add_temp_measurement_cluster(cl, caps_.initial_hp_flow_c, 0.0f, 95.0f);
        }
        if (caps_.has_hp_pump_state) {
            add_binary_input_cluster(cl, "Circulateur Principal PAC", false);
        } else if (caps_.zone1.has_pump_state) {
            add_binary_input_cluster(cl, "Pompe Zone 1 (Q2)", false);
        }
        if (caps_.has_hp_status_code) {
            add_analog_input_cluster(cl, "Code Statut PAC", 0.0f, 0.0f, 255.0f, 1.0f, ZCL_UNIT_DIMENSIONLESS);
        }
        if (caps_.zone1.present) {
            // Entrée AnalogOutput dédiée permettant de lier n'importe quelle sonde de température
            // Home Assistant au Thermostat Zone 1 (0 = désactivé / retour sonde BSB)
            add_analog_output_cluster(cl, "Sonde Temp. Externe Zone 1",
                                      0.0f, 0.0f, 35.0f, 0.1f, ZCL_UNIT_CELSIUS);
        }

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_HYDRAULIC1,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Depart PAC + Pompe Z1 + Sonde Externe Z1)", EP_HYDRAULIC1);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 4 : Retour PAC + Statut Zone 1 + Sonde Temp. Externe Zone 2
    // ------------------------------------------------------------------------
    if (caps_.has_hp_return_temp || caps_.zone1.has_status_code || caps_.zone2.present) {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        if (caps_.has_hp_return_temp) {
            add_temp_measurement_cluster(cl, caps_.initial_hp_return_c, 0.0f, 95.0f);
        }
        if (caps_.zone1.has_status_code) {
            add_analog_input_cluster(cl, "Code Statut Zone 1", 0.0f, 0.0f, 255.0f, 1.0f, ZCL_UNIT_DIMENSIONLESS);
        }
        if (caps_.zone2.present) {
            // Entrée AnalogOutput dédiée permettant de lier n'importe quelle sonde de température
            // Home Assistant au Thermostat Zone 2 (0 = désactivé / retour sonde BSB)
            add_analog_output_cluster(cl, "Sonde Temp. Externe Zone 2",
                                      0.0f, 0.0f, 35.0f, 0.1f, ZCL_UNIT_CELSIUS);
        }

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_HYDRAULIC2,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Retour PAC + Statut Z1 + Sonde Externe Z2)", EP_HYDRAULIC2);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 5 : Température Départ Zone 2 (ou Zone 1) + Consigne Active Z2
    // ------------------------------------------------------------------------
    if (caps_.zone2.has_flow_sensor || caps_.zone1.has_flow_sensor) {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        float init_flow = caps_.zone2.has_flow_sensor ? caps_.zone2.initial_flow_c : caps_.zone1.initial_flow_c;
        add_temp_measurement_cluster(cl, init_flow, 0.0f, 90.0f);

        if (caps_.zone2.present) {
            add_analog_input_cluster(cl, "Consigne Active Zone 2",
                                     caps_.zone2.initial_active_c, 5.0f, 35.0f, 0.1f, ZCL_UNIT_CELSIUS);
        }

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_FLOW_Z2,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Depart Zone + Consigne Active)", EP_FLOW_Z2);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 6 : Ballon Eau Chaude Sanitaire (Thermostat ECS / DHW)
    // ------------------------------------------------------------------------
    if (caps_.has_dhw) {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        // INDEX 2 : Thermostat ECS (strictement aligné sur l'index 2 de l'EP1 et EP2)
        add_dhw_thermostat_cluster(cl,
                                   caps_.initial_dhw_tank_c,
                                   caps_.initial_dhw_setpoint_c,
                                   caps_.initial_dhw_reduced_c,
                                   caps_.initial_dhw_mode);

        if (caps_.has_dhw_tank_temp) {
            add_temp_measurement_cluster(cl, caps_.initial_dhw_tank_c, 5.0f, 85.0f);
        }

        add_analog_output_cluster(cl, "Consigne Ballon ECS",
                                  caps_.initial_dhw_setpoint_c, 30.0f, 65.0f, 0.5f, ZCL_UNIT_CELSIUS);

        esp_zb_on_off_cluster_cfg_t on_off_cfg = {
            .on_off = (caps_.initial_dhw_mode != static_cast<uint8_t>(bsb::DhwMode::Off)),
        };
        esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // Code Statut ECS (8003) toujours exposé dès que le ballon ECS est actif
        add_analog_input_cluster(cl, "Code Statut ECS", 0.0f, 0.0f, 255.0f, 1.0f, ZCL_UNIT_DIMENSIONLESS);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_DHW,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Thermostat Ballon ECS / DHW)", EP_DHW);
    }

    // ------------------------------------------------------------------------
    // ENDPOINT 7 : Appoint Électrique + Sonde Ambiance BSB Zone 1 + Switch Antenne Externe
    // ------------------------------------------------------------------------
    {
        esp_zb_cluster_list_t* cl = esp_zb_zcl_cluster_list_create();
        add_basic_and_identify_clusters(cl, caps_.controller_model, false);

        if (caps_.has_backup_heater) {
            add_binary_input_cluster(cl, "Appoint Electrique", false);
        }
        if (caps_.has_backup_mod) {
            add_analog_input_cluster(cl, "Modulation Appoint", 0.0f, 0.0f, 100.0f, 1.0f, ZCL_UNIT_PERCENT);
        }
        if (caps_.zone1.has_room_sensor) {
            add_temp_measurement_cluster(cl, caps_.zone1.initial_room_c, 0.0f, 45.0f);
        }

        // Commutateur d'antenne RF Zigbee du XIAO ESP32-C6 (OFF = Céramique interne par défaut, ON = U.FL externe)
        esp_zb_on_off_cluster_cfg_t ant_on_off_cfg = {
            .on_off = ext_antenna_enabled_,
        };
        esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&ant_on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint           = EP_BACKUP_AUX,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
        ESP_LOGI(TAG, "Endpoint %u enregistre (Appoint Electrique, Auxiliaires & Switch Antenne RF=%s)",
                 EP_BACKUP_AUX, ext_antenna_enabled_ ? "ON (Externe U.FL)" : "OFF (Interne Ceramique)");
    }
}

// ============================================================================
// Verrou de la pile Zigbee & utilitaires
// ============================================================================

namespace {

TaskHandle_t s_zb_task = nullptr;

// Vrai une fois esp_zb_start() terminé : avant cela, la tâche Arduino ne touche à aucun attribut ZCL
// (la pile charge sa NVRAM hors verrou pendant son démarrage).
std::atomic<bool> s_stack_running{false};

/**
 * Verrou RAII de la pile Zigbee. Dans la tâche Zigbee (callbacks de la pile), le verrou est déjà
 * tenu par la boucle principale de la pile : rien n'est pris ni relâché.
 */
class ZbLock {
public:
    explicit ZbLock(TickType_t ticks = pdMS_TO_TICKS(1000)) {
        if (s_zb_task != nullptr && xTaskGetCurrentTaskHandle() == s_zb_task) {
            held_ = true;
        } else {
            owned_ = esp_zb_lock_acquire(ticks);
            held_  = owned_;
        }
    }
    ~ZbLock() {
        if (owned_) {
            esp_zb_lock_release();
        }
    }
    ZbLock(const ZbLock&)            = delete;
    ZbLock& operator=(const ZbLock&) = delete;
    explicit operator bool() const { return held_; }

private:
    bool held_  = false;
    bool owned_ = false;
};

/** Lecture d'une valeur d'attribut ZCL sans hypothèse d'alignement du tampon de la pile. */
template <typename T>
T read_value(const void* data) {
    T value;
    memcpy(&value, data, sizeof(T));
    return value;
}

uint8_t clamp_percent(int32_t value) {
    if (value < 0) return 0;
    if (value > 100) return 100;
    return static_cast<uint8_t>(value);
}

uint16_t clamp_status_code(int32_t value) {
    if (value < 0) return 0;
    if (value > 0xFFFF) return 0xFFFF;
    return static_cast<uint16_t>(value);
}

bool is_thermostat_attr_reportable(uint16_t attr_id) {
    switch (attr_id) {
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID:
            return true;
        default:
            return false; // Non reportable dans ZBOSS standard : exposé via AnalogOutput ou lu à la demande
    }
}

} // namespace

// ============================================================================
// Callbacks C globaux pour la pile ZBOSS / esp-zigbee-sdk
// ============================================================================

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void* message) {
    if (!message) return ESP_OK;

    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return ZbBridge::instance().handle_attribute_write(
            static_cast<const esp_zb_zcl_set_attr_value_message_t*>(message));
    }
    if (callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        return ZbBridge::instance().handle_attribute_report(
            static_cast<const esp_zb_zcl_report_attr_message_t*>(message));
    }
    if (callback_id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) {
        return ZbBridge::instance().handle_ota_upgrade_status(
            static_cast<const esp_zb_zcl_ota_upgrade_value_message_t*>(message));
    }
    if (callback_id == ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID) {
        return ZbBridge::instance().handle_ota_query_image_resp(
            static_cast<const esp_zb_zcl_ota_upgrade_query_image_resp_message_t*>(message));
    }
    return ESP_OK;
}

extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
    ZbBridge::instance().handle_stack_signal(signal_struct);
}

static void bdb_start_commissioning_cb(uint8_t mode_mask) {
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

static void zigbee_main_task(void* pvParameters) {
    s_zb_task = xTaskGetCurrentTaskHandle();
    esp_zb_bdb_set_scan_duration(3);
    // Puissance d'émission maximale (+20 dBm) pour un rôle Routeur Zigbee optimal
    esp_zb_set_tx_power(20);
    ESP_ERROR_CHECK(esp_zb_start(false));
    s_stack_running.store(true);
    esp_zb_stack_main_loop();
}

// ============================================================================
// Initialisation, antenne RF & disposition Zigbee
// ============================================================================

bool ZbBridge::load_external_antenna_nvs() {
    nvs_handle_t nvs_h = 0;
    uint8_t val = 0; // Par défaut : OFF (0 = antenne céramique interne)
    if (nvs_open("bsb_topo", NVS_READONLY, &nvs_h) == ESP_OK) {
        nvs_get_u8(nvs_h, "ext_ant", &val);
        nvs_close(nvs_h);
    }
    return (val != 0);
}

void ZbBridge::set_external_antenna_enabled(bool enabled) {
    ext_antenna_enabled_ = enabled;
    board::set_xiao_rf_antenna(enabled);

    nvs_handle_t nvs_h = 0;
    if (nvs_open("bsb_topo", NVS_READWRITE, &nvs_h) == ESP_OK) {
        nvs_set_u8(nvs_h, "ext_ant", enabled ? 1 : 0);
        nvs_commit(nvs_h);
        nvs_close(nvs_h);
    }

    ESP_LOGI(TAG, "[Zigbee->RF] Commutateur d'antenne RF XIAO ESP32-C6 -> %s (sauvegarde en NVS)",
             enabled ? "ON (Antenne Externe U.FL)" : "OFF (Antenne Interne Ceramique)");

    if (s_stack_running.load()) {
        ZbLock lock;
        if (lock) {
            set_on_off_attr(EP_BACKUP_AUX, enabled);
        } else {
            zb_lock_timeouts_++;
        }
    }
}

uint32_t ZbBridge::layout_signature(const bsb::BsbCapabilities& caps) {
    // Chaque drapeau ci-dessous ajoute ou retire un endpoint ou un cluster dans build_endpoints()
    const bool flags[] = {
        caps.has_outdoor_temp,      caps.has_compressor_state,  caps.has_compressor_mod,
        caps.zone1.present,         caps.zone1.has_room_sensor, caps.zone1.has_flow_sensor,
        caps.zone1.has_pump_state,  caps.zone1.has_status_code,
        caps.zone2.present,         caps.zone2.has_room_sensor, caps.zone2.has_flow_sensor,
        caps.zone2.has_pump_state,  caps.zone2.has_status_code,
        caps.has_hp_flow_temp,      caps.has_hp_return_temp,    caps.has_hp_pump_state,
        caps.has_hp_status_code,    caps.has_dhw,               caps.has_dhw_tank_temp,
        caps.has_backup_heater,     caps.has_backup_mod,
    };
    uint32_t signature = 0;
    uint32_t bit       = 0;
    for (const bool flag : flags) {
        if (flag) {
            signature |= (1UL << bit);
        }
        ++bit;
    }
    return signature;
}

bool ZbBridge::has_ep_hydraulic1() const {
    return caps_.has_hp_flow_temp || caps_.zone1.present || caps_.has_hp_status_code;
}

bool ZbBridge::has_ep_hydraulic2() const {
    return caps_.has_hp_return_temp || caps_.zone1.has_status_code || caps_.zone2.present;
}

bool ZbBridge::has_ep_flow() const {
    return caps_.zone2.has_flow_sensor || caps_.zone1.has_flow_sensor;
}

bool ZbBridge::begin(const bsb::BsbCapabilities& caps, bsb::BsbBus* bus, bool erase_zigbee_nvram) {
    caps_                = caps;
    bus_                 = bus;
    z1_mode_             = caps.zone1.initial_mode;
    z2_mode_             = caps.zone2.initial_mode;
    dhw_mode_            = caps.initial_dhw_mode;
    // Valeurs de SystemMode avec lesquelles build_endpoints() crée les trois thermostats
    zcl_mode_z1_         = bsb_mode_to_zcl_system_mode(caps.zone1.initial_mode);
    zcl_mode_z2_         = bsb_mode_to_zcl_system_mode(caps.zone2.initial_mode);
    zcl_mode_dhw_        = dhw_mode_to_zcl_system_mode(caps.initial_dhw_mode);
    ext_antenna_enabled_ = load_external_antenna_nvs();
    board::set_xiao_rf_antenna(ext_antenna_enabled_);
    track_measurements(now_ms());

    esp_zb_platform_config_t platform_config = {};
    platform_config.radio_config.radio_mode          = ZB_RADIO_MODE_NATIVE;
    platform_config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;

    esp_err_t err = esp_zb_platform_config(&platform_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec esp_zb_platform_config : %s", esp_err_to_name(err));
        return false;
    }

    // Configuration en Routeur Zigbee 3.0 (alimenté sur secteur, répéteur mesh actif)
    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = 16;

    // Table de liaisons : une liaison par couple (endpoint, cluster) est demandée par le coordinateur à
    // l'appairage, soit près d'une trentaine pour une installation complète. Avec les 16 entrées par
    // défaut, les clusters des derniers endpoints n'étaient pas liés (pas de rapports natifs de la pile).
    esp_zb_aps_src_binding_table_size_set(BINDING_TABLE_SIZE);
    esp_zb_aps_dst_binding_table_size_set(BINDING_TABLE_SIZE);

    esp_zb_init(&zb_nwk_cfg);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    if (erase_zigbee_nvram) {
        ESP_LOGW(TAG, "Effacement NVRAM Zigbee demande (re-appairage)...");
        esp_zb_nvram_erase_at_start(true);
    }

    esp_zb_ep_list_t* ep_list = esp_zb_ep_list_create();
    build_endpoints(ep_list);

    err = esp_zb_device_register(ep_list);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec esp_zb_device_register : %s", esp_err_to_name(err));
        return false;
    }
    esp_zb_core_action_handler_register(zb_action_handler);

    err = esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec esp_zb_set_primary_network_channel_set : %s", esp_err_to_name(err));
        return false;
    }

    stack_initialized_ = true;

    if (xTaskCreate(zigbee_main_task, "zb_main", 8192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Echec de creation de la tache Zigbee");
        stack_initialized_ = false;
        return false;
    }
    return true;
}

bool ZbBridge::is_stack_started() const {
    return s_stack_running.load();
}

void ZbBridge::factory_reset() {
    if (!stack_initialized_) {
        return;
    }
    ZbLock lock(portMAX_DELAY);
    esp_zb_factory_reset(); // Efface zb_storage puis redémarre l'appareil
}

// ============================================================================
// Signaux de la pile Zigbee (tâche Zigbee)
// ============================================================================

void ZbBridge::on_network_joined() {
    awaiting_pairing_.store(false);
    connected_.store(true);
    // L'état de chauffe sera journalisé de nouveau à la prochaine publication
    for (HeatingPublication* pub : {&z1_heating_, &z2_heating_, &dhw_heating_}) {
        pub->last_heating = -1;
        pub->last_demand  = -1;
    }
    request_ota_update();
    force_report_all();
}

void ZbBridge::handle_stack_signal(esp_zb_app_signal_t* signal_struct) {
    if (!signal_struct || !signal_struct->p_app_signal) {
        return;
    }
    uint32_t* p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status   = signal_struct->esp_err_status;
    auto      sig_type     = static_cast<esp_zb_app_signal_type_t>(*p_sg_p);

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Pile Zigbee initialisee, demarrage BDB...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Appareil neuf : recherche d'un coordinateur Zigbee (Steering)...");
                    awaiting_pairing_.store(true);
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Reconnecte au reseau Zigbee (PAN=0x%04X, Canal=%u, Addr=0x%04X)",
                             esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                    on_network_joined();
                }
            } else {
                ESP_LOGW(TAG, "Echec initialisation BDB (%s), nouvel essai dans 1s...", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Appairage Zigbee reussi ! (PAN=0x%04X, Canal=%u, Addr=0x%04X)",
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                on_network_joined();
            } else {
                connected_.store(false);
                ESP_LOGI(TAG, "En attente d'ouverture du reseau Zigbee (ZHA / Z2M)... (%s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            const auto* leave = static_cast<const esp_zb_zdo_signal_leave_params_t*>(
                esp_zb_app_signal_get_params(p_sg_p));
            connected_.store(false);
            if (leave && leave->leave_type == ESP_ZB_NWK_LEAVE_TYPE_REJOIN) {
                // Le coordinateur demande une ré-association : la pile rejoint le réseau au redémarrage
                ESP_LOGW(TAG, "Sortie du reseau Zigbee avec re-association demandee : redemarrage...");
                esp_restart();
            } else {
                // Retrait définitif : repartir d'un état usine propre (bindings et reporting purgés),
                // l'appareil redémarre ensuite en mode appairage.
                ESP_LOGW(TAG, "Appareil retire du reseau Zigbee : reinitialisation usine Zigbee et re-appairage...");
                esp_zb_factory_reset();
            }
            break;
        }

        default:
            break;
    }
}

// ============================================================================
// Reporting ZCL vers le coordinateur (verrou Zigbee tenu par l'appelant)
// ============================================================================

ZbBridge::ReportCacheEntry* ZbBridge::find_cache_entry(uint8_t ep, uint16_t cluster_id, uint16_t attr_id) {
    for (size_t i = 0; i < report_cache_count_; ++i) {
        ReportCacheEntry& e = report_cache_[i];
        if (e.ep == ep && e.cluster_id == cluster_id && e.attr_id == attr_id) {
            return &e;
        }
    }
    return nullptr;
}

void ZbBridge::remember_value(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric) {
    ReportCacheEntry* e = find_cache_entry(ep, cluster_id, attr_id);
    if (!e) {
        if (report_cache_count_ >= MAX_REPORT_CACHE) {
            return; // Cache plein : cet attribut sera simplement ré-émis à chaque mise à jour
        }
        e             = &report_cache_[report_cache_count_++];
        e->ep         = ep;
        e->cluster_id = cluster_id;
        e->attr_id    = attr_id;
    }
    e->value          = numeric;
    e->last_report_ms = now_ms();
}

void ZbBridge::forget_value(uint8_t ep, uint16_t cluster_id, uint16_t attr_id) {
    ReportCacheEntry* e = find_cache_entry(ep, cluster_id, attr_id);
    if (!e) {
        return;
    }
    // Suppression en O(1) : la dernière entrée prend la place libérée
    *e = report_cache_[report_cache_count_ - 1];
    report_cache_count_--;
}

void ZbBridge::schedule_republish(uint8_t ep, uint16_t cluster_id, uint16_t attr_id) {
    for (size_t i = 0; i < pending_republish_count_; ++i) {
        const PendingRepublish& p = pending_republish_[i];
        if (p.ep == ep && p.cluster_id == cluster_id && p.attr_id == attr_id) {
            return; // Déjà demandé
        }
    }
    if (pending_republish_count_ < MAX_PENDING_REPUBLISH) {
        pending_republish_[pending_republish_count_++] = {ep, cluster_id, attr_id};
    }
}

void ZbBridge::flush_pending_republish() {
    for (size_t i = 0; i < pending_republish_count_; ++i) {
        const PendingRepublish& p = pending_republish_[i];
        // Le cache est oublié dans tous les cas : si le rapport ne peut pas partir maintenant, la
        // prochaine publication de cet attribut ne sera pas prise pour une valeur déjà connue.
        forget_value(p.ep, p.cluster_id, p.attr_id);
        if (!connected_.load() || ota_in_progress_.load() || !send_report(p.ep, p.cluster_id, p.attr_id)) {
            continue;
        }
        const esp_zb_zcl_attr_t* attr =
            esp_zb_zcl_get_attribute(p.ep, p.cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, p.attr_id);
        if (!attr || !attr->data_p) {
            continue;
        }
        float numeric = 0.0f;
        switch (attr->type) {
            case ESP_ZB_ZCL_ATTR_TYPE_SINGLE:
                memcpy(&numeric, attr->data_p, sizeof(numeric));
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_S16: {
                int16_t v = 0;
                memcpy(&v, attr->data_p, sizeof(v));
                numeric = static_cast<float>(v);
                break;
            }
            case ESP_ZB_ZCL_ATTR_TYPE_U16:
            case ESP_ZB_ZCL_ATTR_TYPE_16BITMAP: {
                uint16_t v = 0;
                memcpy(&v, attr->data_p, sizeof(v));
                numeric = static_cast<float>(v);
                break;
            }
            default: // Types sur un octet : booléen, énumération, entier ou champ de bits 8 bits
                numeric = static_cast<float>(*static_cast<const uint8_t*>(attr->data_p));
                break;
        }
        remember_value(p.ep, p.cluster_id, p.attr_id, numeric);
    }
    pending_republish_count_ = 0;
}

bool ZbBridge::send_report(uint8_t ep, uint16_t cluster_id, uint16_t attr_id) {
    esp_zb_zcl_attr_location_info_t info = {
        .endpoint_id  = ep,
        .cluster_id   = cluster_id,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .manuf_code   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .attr_id      = attr_id,
    };

    // S'assurer qu'une entrée de reporting existe dans ZBOSS (profil HA) pour cet attribut
    esp_zb_zcl_reporting_info_t* rep_ptr = esp_zb_zcl_find_reporting_info(info);
    if (!rep_ptr || rep_ptr->dst.profile_id == 0) {
        esp_zb_zcl_reporting_info_t rep = {};
        rep.direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
        rep.ep           = ep;
        rep.cluster_id   = cluster_id;
        rep.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        rep.attr_id      = attr_id;
        rep.manuf_code   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

        rep.u.send_info.min_interval     = 1;
        rep.u.send_info.max_interval     = 300;
        rep.u.send_info.def_min_interval = 1;
        rep.u.send_info.def_max_interval = 300;

        rep.dst.short_addr = 0x0000;
        rep.dst.endpoint   = 1;
        rep.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;

        esp_zb_zcl_update_reporting_info(&rep);
    }

    esp_zb_zcl_report_attr_cmd_t cmd = {};
    cmd.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.dst_endpoint          = 1;
    cmd.zcl_basic_cmd.src_endpoint          = ep;
    cmd.clusterID                           = cluster_id;
    cmd.attributeID                         = attr_id;
    cmd.direction                           = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.manuf_specific                      = 0;
    cmd.dis_default_resp                    = 1;

    const esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&cmd);
    if (ret != ESP_OK) {
        // Journalisation espacée : une saturation passagère de la pile ne doit pas inonder la console
        if ((zb_reports_failed_++ % 20) == 0) {
            ESP_LOGW(TAG, "Echec d'emission du rapport EP%u cluster 0x%04X attr 0x%04X : %s",
                     ep, cluster_id, attr_id, esp_err_to_name(ret));
        }
        return false;
    }
    zb_reports_sent_++;
    return true;
}

void ZbBridge::force_report(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric) {
    if (!connected_.load() || ota_in_progress_.load()) {
        forget_value(ep, cluster_id, attr_id);
        return;
    }
    if (send_report(ep, cluster_id, attr_id)) {
        remember_value(ep, cluster_id, attr_id, numeric);
    } else {
        forget_value(ep, cluster_id, attr_id);
    }
}

bool ZbBridge::write_attr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, void* value) {
    const esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(ep, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                                    attr_id, value, false);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        if ((zb_attr_errors_++ % 50) == 0) {
            ESP_LOGW(TAG, "Ecriture attribut refusee : EP%u cluster 0x%04X attr 0x%04X (statut ZCL 0x%02X)",
                     ep, cluster_id, attr_id, static_cast<unsigned>(status));
        }
        return false;
    }
    return true;
}

void ZbBridge::publish(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, void* value, float numeric,
                       float min_delta, uint32_t heartbeat_ms) {
    if (!write_attr(ep, cluster_id, attr_id, value)) {
        return;
    }
    report_if_changed(ep, cluster_id, attr_id, numeric, min_delta, heartbeat_ms);
}

void ZbBridge::report_if_changed(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric,
                                 float min_delta, uint32_t heartbeat_ms) {
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT && !is_thermostat_attr_reportable(attr_id)) {
        return;
    }
    if (!connected_.load() || ota_in_progress_.load()) {
        return; // Rien n'est noté dans le cache : la valeur partira dès que le lien sera disponible
    }

    // Déduplication : on n'émet que si la valeur a suffisamment changé depuis le dernier rapport
    // réellement transmis, ou si le heartbeat de cet attribut est échu. Le passage d'une valeur à
    // « indisponible » (NaN) et le retour à une valeur comptent toujours comme un changement : une
    // différence avec NaN n'est jamais « assez grande » au sens de la comparaison numérique.
    if (const ReportCacheEntry* e = find_cache_entry(ep, cluster_id, attr_id)) {
        const bool changed = (std::isnan(numeric) != std::isnan(e->value)) ||
                             std::fabs(numeric - e->value) >= min_delta;
        const bool stale   = elapsed_ms(now_ms(), e->last_report_ms) >= heartbeat_ms;
        if (!changed && !stale) {
            zb_reports_suppressed_++;
            return;
        }
    }

    if (send_report(ep, cluster_id, attr_id)) {
        remember_value(ep, cluster_id, attr_id, numeric);
    }
}

/**
 * Borne une valeur en centièmes de degré entre deux attributs « limite » du même cluster. La pile ZBOSS
 * refuse (statut INVALID_VALUE) toute écriture locale d'une mesure ou d'une consigne en dehors des bornes
 * déclarées, même avec check=false : sans cette borne, la valeur resterait figée dans Home Assistant.
 */
static int16_t clamp_to_attr_limits(uint8_t ep, uint16_t cluster_id, uint16_t min_attr_id, uint16_t max_attr_id,
                                    int16_t value) {
    const esp_zb_zcl_attr_t* min_attr = esp_zb_zcl_get_attribute(ep, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, min_attr_id);
    const esp_zb_zcl_attr_t* max_attr = esp_zb_zcl_get_attribute(ep, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, max_attr_id);
    if (min_attr && min_attr->data_p) {
        int16_t min_val = 0;
        memcpy(&min_val, min_attr->data_p, sizeof(min_val));
        if (min_val != INT16_MIN && value < min_val) {
            value = min_val;
        }
    }
    if (max_attr && max_attr->data_p) {
        int16_t max_val = 0;
        memcpy(&max_val, max_attr->data_p, sizeof(max_val));
        if (max_val != INT16_MIN && value > max_val) {
            value = max_val;
        }
    }
    return value;
}

void ZbBridge::set_temp_meas_attr(uint8_t ep, float temp_c) {
    int16_t val = temp_to_zcl_s16(temp_c);
    if (val != INT16_MIN) {
        val = clamp_to_attr_limits(ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                   ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,
                                   ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, val);
    }
    // Émission si variation >= 0.10 °C
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            &val, static_cast<float>(val), 10.0f);
}

void ZbBridge::set_binary_input_attr(uint8_t ep, bool state) {
    uint8_t val = state ? 1 : 0;
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
            &val, static_cast<float>(val), 0.5f);
}

void ZbBridge::set_analog_input_attr(uint8_t ep, float val, uint32_t heartbeat_ms) {
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            &val, val, 0.5f, heartbeat_ms);
}

void ZbBridge::set_analog_output_attr(uint8_t ep, float val) {
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
            &val, val, 0.1f);
}

void ZbBridge::set_on_off_attr(uint8_t ep, bool on) {
    uint8_t val = on ? 1 : 0;
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            &val, static_cast<float>(val), 0.5f);
}

/**
 * Pose une valeur dans le stockage d'un attribut 16 bits signé, sans passer par le contrôle de plage de la
 * pile. Réservé à la valeur « mesure indisponible » (0x8000) des températures du cluster Thermostat : la
 * spécification ZCL la définit pour cet usage, mais ZBOSS la refuse à l'écriture ordinaire (INVALID_VALUE).
 */
static bool store_s16_attr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, int16_t value) {
    const esp_zb_zcl_attr_t* attr = esp_zb_zcl_get_attribute(ep, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id);
    if (!attr || !attr->data_p || attr->type != ESP_ZB_ZCL_ATTR_TYPE_S16) {
        return false;
    }
    memcpy(attr->data_p, &value, sizeof(value));
    return true;
}

void ZbBridge::set_thermostat_temp_attr(uint8_t ep, uint16_t attr_id, float temp_c) {
    int16_t val = temp_to_zcl_s16(temp_c);
    if (val == INT16_MIN) {
        // Mesure que le régulateur ne fournit plus : le rapport et les lectures du coordinateur reprennent
        // la valeur invalide depuis le stockage de l'attribut.
        if (store_s16_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, val)) {
            report_if_changed(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, static_cast<float>(val), 10.0f,
                              DEFAULT_HEARTBEAT_MS);
        }
        return;
    }
    if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID ||
        attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID) {
        // Une consigne réglée sur la PAC en dehors des limites exposées (7-30 °C / 30-65 °C) s'affiche à la
        // limite la plus proche plutôt que de rester figée sur son ancienne valeur.
        val = clamp_to_attr_limits(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                   ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,
                                   ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, val);
    }
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, &val, static_cast<float>(val), 10.0f);
}

uint8_t& ZbBridge::tracked_system_mode(uint8_t ep) {
    return (ep == EP_MASTER_Z1) ? zcl_mode_z1_ : (ep == EP_ZONE2) ? zcl_mode_z2_ : zcl_mode_dhw_;
}

void ZbBridge::set_thermostat_system_mode(uint8_t ep, uint8_t zcl_mode, bool from_coordinator_write) {
    const uint16_t cluster = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
    const uint16_t attr    = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID;
    uint8_t&       tracked = tracked_system_mode(ep);

    if (zcl_mode == ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF &&
        (tracked != ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF || from_coordinator_write)) {
        // La pile ZBOSS refuse toute écriture de PIHeatingDemand tant que SystemMode vaut Off. La demande
        // et l'état de marche sont donc remis à zéro AVANT de passer en Off (en repassant brièvement par
        // Heat si le coordinateur vient déjà d'écrire Off), sinon Home Assistant resterait « en chauffe ».
        uint8_t heat = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;
        write_attr(ep, cluster, attr, &heat);
        set_thermostat_u8_attr(ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, 0, STATE_HEARTBEAT_MS);
        set_thermostat_u16_attr(ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, 0x0000, STATE_HEARTBEAT_MS);
    }
    tracked = zcl_mode;

    if (from_coordinator_write) {
        // L'attribut reflète le mode réellement appliqué, pas forcément la valeur brute écrite
        if (write_attr(ep, cluster, attr, &zcl_mode)) {
            force_report(ep, cluster, attr, static_cast<float>(zcl_mode));
        }
    } else {
        set_thermostat_u8_attr(ep, attr, zcl_mode, STATE_HEARTBEAT_MS);
    }
}

void ZbBridge::set_thermostat_u8_attr(uint8_t ep, uint16_t attr_id, uint8_t val, uint32_t heartbeat_ms) {
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, &val, static_cast<float>(val), 0.5f, heartbeat_ms);
}

void ZbBridge::set_thermostat_u16_attr(uint8_t ep, uint16_t attr_id, uint16_t val, uint32_t heartbeat_ms) {
    publish(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, &val, static_cast<float>(val), 0.5f, heartbeat_ms);
}

// ============================================================================
// Gestion des sondes d'ambiance externes (Home Assistant ou Binding Zigbee)
// ============================================================================

void ZbBridge::deactivate_external_sensor(uint8_t zone_index, const char* reason) {
    const bool is_z2           = (zone_index == 2);
    ExternalSensorState& state = is_z2 ? ext_sensor_z2_ : ext_sensor_z1_;
    if (!state.active) {
        return;
    }
    state.active    = false;
    state.confirmed = false;
    state.warned    = false;

    set_thermostat_u8_attr(is_z2 ? EP_ZONE2 : EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, 0x00);
    // 0 °C sur l'entité `number` = sonde externe désactivée : Home Assistant voit le retour sur la mesure BSB
    set_analog_output_attr(is_z2 ? EP_HYDRAULIC2 : EP_HYDRAULIC1, 0.0f);
    ESP_LOGW(TAG, "Sonde d'ambiance externe Zone %u desactivee (%s) -> retour sur mesure BSB.", zone_index, reason);
}

void ZbBridge::set_external_room_temperature(uint8_t zone_index, float temp_c, bool from_binding) {
    if (!bus_ || (zone_index != 1 && zone_index != 2)) return;

    const bool     is_z2       = (zone_index == 2);
    const uint8_t  thermo_ep   = is_z2 ? EP_ZONE2 : EP_MASTER_Z1;
    const uint8_t  ao_ep       = is_z2 ? EP_HYDRAULIC2 : EP_HYDRAULIC1;
    const uint32_t inf_field   = is_z2 ? bsb::fields::Z2_ROOM_TEMP_INF : bsb::fields::Z1_ROOM_TEMP_INF;
    ExternalSensorState& state = is_z2 ? ext_sensor_z2_ : ext_sensor_z1_;

    if (!(is_z2 ? caps_.zone2.present : caps_.zone1.present)) return;

    if (std::isfinite(temp_c) && temp_c < EXT_SENSOR_MIN_C) {
        // Une valeur < 4.0 °C (ex: 0.0 °C) désactive la sonde externe et rend la main au bus BSB
        deactivate_external_sensor(zone_index, "valeur de desactivation recue");
        // Désactivation voulue par l'utilisateur : la zone n'est plus marquée « alimentée par une sonde
        // externe » (un simple silence de Home Assistant, lui, ne l'efface pas).
        ext_used_request_[is_z2 ? 1 : 0] = 0;
    } else if (std::isfinite(temp_c) && temp_c <= EXT_SENSOR_MAX_C) {
        const uint64_t t_now = now_ms();
        if (!state.active) {
            state.active       = true;
            state.confirmed    = false;
            state.warned       = false;
            state.activated_ms = t_now;
            if (!(is_z2 ? caps_.zone2.has_room_sensor : caps_.zone1.has_room_sensor)) {
                // Zone sans sonde filaire : le régulateur restituera désormais notre valeur sur 8740 / 8770.
                // Une future redécouverte ne devra pas la prendre pour une sonde filaire.
                ext_used_request_[is_z2 ? 1 : 0] = 1;
            }
        }
        state.from_binding    = from_binding;
        state.temperature_c   = temp_c;
        state.last_update_ms  = t_now;
        state.last_bsb_inf_ms = t_now;

        // La source est journalisée : deux sources actives à la fois (un capteur lié en direct ET une
        // automatisation) se reconnaissent à des valeurs qui alternent.
        ESP_LOGI(TAG, "[Sonde Externe Zone %u] Temp. Ambiante = %.2f C (%s) -> Thermostat EP%u & Bus BSB (INF 0x%08lX)",
                 zone_index, temp_c, from_binding ? "capteur lie en direct" : "entite number",
                 thermo_ep, static_cast<unsigned long>(inf_field));

        // 1. LocalTemperature et bit RemoteSensing du Thermostat Zigbee de la zone
        set_thermostat_temp_attr(thermo_ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, temp_c);
        set_thermostat_u8_attr(thermo_ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID,
                               ESP_ZB_ZCL_THERMOSTAT_REMOTE_SENSING_LOCAL_TEMPERATURE_SENSED_REMOTELY_BIT);

        // 2. Transmettre immédiatement la température d'ambiance au régulateur (trame INF)
        if (!bus_->enqueue_inf_room_temp(inf_field, temp_c)) {
            ESP_LOGW(TAG, "File d'ecriture BSB pleine : temperature externe Zone %u non transmise", zone_index);
        }
    }
    // Valeur non exploitable (NaN, > 38 °C) : ignorée, la sonde externe garde son état précédent.

    // 3. L'entité `number` affiche toujours la valeur effectivement utilisée (0 = désactivée)
    set_analog_output_attr(ao_ep, state.active ? state.temperature_c : 0.0f);
}

void ZbBridge::maintain_external_sensor(uint8_t zone_index, uint64_t t_now) {
    const bool is_z2           = (zone_index == 2);
    ExternalSensorState& state = is_z2 ? ext_sensor_z2_ : ext_sensor_z1_;
    if (!state.active) {
        return;
    }

    const uint64_t watchdog_ms = state.from_binding ? EXT_SENSOR_BINDING_WATCHDOG_MS
                                                    : EXT_SENSOR_WATCHDOG_TIMEOUT_MS;
    if (elapsed_ms(t_now, state.last_update_ms) >= watchdog_ms) {
        deactivate_external_sensor(zone_index, state.from_binding ? "aucun rapport du capteur lie depuis 3 h"
                                                                  : "aucune mise a jour depuis 30 min");
        return;
    }

    if (elapsed_ms(t_now, state.last_bsb_inf_ms) >= EXT_SENSOR_BSB_REBROADCAST_MS) {
        state.last_bsb_inf_ms = t_now;
        bus_->enqueue_inf_room_temp(is_z2 ? bsb::fields::Z2_ROOM_TEMP_INF : bsb::fields::Z1_ROOM_TEMP_INF,
                                    state.temperature_c);
    }

    // La prise en compte par le régulateur se vérifie en continu : sans relecture concordante depuis
    // 5 min (une réponse « --- » ou une autre mesure ne concordent pas), elle n'est plus acquise.
    const uint64_t reference_ms = state.confirmed ? state.last_match_ms : state.activated_ms;
    if (elapsed_ms(t_now, reference_ms) >= EXT_SENSOR_CONFIRM_TIMEOUT_MS) {
        if (state.confirmed) {
            state.confirmed = false;
            state.warned    = false;
        }
        if (!state.warned) {
            state.warned = true;
            ESP_LOGW(TAG, "Sonde externe Zone %u : le regulateur ne relit pas la temperature injectee (Prog %s) "
                          "-> verifier qu'une sonde d'ambiance est autorisee sur ce circuit.",
                     zone_index, is_z2 ? "8770" : "8740");
        }
    }
}

void ZbBridge::check_external_sensor_feedback(uint8_t zone_index, float controller_room_c) {
    ExternalSensorState& state = (zone_index == 2) ? ext_sensor_z2_ : ext_sensor_z1_;
    if (!state.active) {
        return;
    }
    if (std::fabs(controller_room_c - state.temperature_c) <= 0.3f) {
        state.last_match_ms = now_ms();
        if (!state.confirmed) {
            state.confirmed = true;
            state.warned    = false;
            ESP_LOGI(TAG, "Sonde externe Zone %u prise en compte par le regulateur (temperature ambiante relue = %.2f C)",
                     zone_index, controller_room_c);
        }
    }
}

void ZbBridge::loop_housekeeping() {
    if (!s_stack_running.load() || !bus_) return;

    // Cadence d'une seconde (horodatages propres à la tâche Arduino)
    const uint64_t t_entry = now_ms();
    if (elapsed_ms(t_entry, last_housekeeping_ms_) < 1000ULL) {
        return;
    }
    last_housekeeping_ms_ = t_entry;

    ZbLock lock(pdMS_TO_TICKS(100));
    if (!lock) {
        return; // Pile occupée : nouvelle tentative à la prochaine seconde
    }

    // L'horloge est relue APRÈS l'obtention du verrou : pendant l'attente, la tâche Zigbee a pu
    // horodater un bloc OTA ou une température externe plus récents que la lecture d'entrée.
    const uint64_t t_now = now_ms();

    // Relecture complète demandée après un (ré)appairage : seule cette tâche a le droit de piloter le bus
    if (resync_requested_.exchange(false)) {
        bus_->refresh_all();
    }

    // Consigne réduite ECS refusée (paramètre absent du régulateur) : la valeur d'origine est republiée
    // ici, hors du callback d'écriture et après la réponse au coordinateur.
    if (dhw_reduced_restore_pending_) {
        dhw_reduced_restore_pending_ = false;
        forget_value(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                     ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID);
        set_thermostat_temp_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID,
                                 caps_.initial_dhw_reduced_c);
    }

    // Valeurs corrigées lors d'une écriture du coordinateur : republiées maintenant, après sa réponse
    flush_pending_republish();

    // Maintenance des sondes externes (ré-émission INF 60 s, watchdog 30 min)
    maintain_external_sensor(1, t_now);
    maintain_external_sensor(2, t_now);

    // Mémorisation durable « zone alimentée par une sonde externe » (une écriture NVS par changement)
    for (uint8_t i = 0; i < 2; ++i) {
        if (ext_used_request_[i] >= 0) {
            bsb::BsbDiscovery::set_external_sensor_used(static_cast<uint8_t>(i + 1), ext_used_request_[i] == 1);
            ext_used_request_[i] = -1;
        }
    }

    // Resynchronisation périodique de l'état de chauffe des thermostats (Running State & Demand)
    if (elapsed_ms(t_now, last_running_sync_ms_) >= 5000ULL) {
        last_running_sync_ms_ = t_now;
        update_thermostat_running_state();
        expire_stale_measurements(t_now);
    }

    // Transfert OTA resté sans nouvelles : sans cette garde, le reporting resterait suspendu indéfiniment
    if (ota_in_progress_.load() && elapsed_ms(t_now, s_ota_last_activity_ms) >= OTA_STALL_TIMEOUT_MS) {
        ESP_LOGW(TAG, "[OTA] Aucun bloc recu depuis %lu min : transfert considere comme abandonne.",
                 static_cast<unsigned long>(OTA_STALL_TIMEOUT_MS / 60000ULL));
        ota_abort_transfer();
        ota_in_progress_.store(false);
    }
}

// ============================================================================
// Réception d'un rapport direct par Binding Zigbee (Capteur Temp -> EP1 / EP2)
// ============================================================================

esp_err_t ZbBridge::handle_attribute_report(const esp_zb_zcl_report_attr_message_t* msg) {
    if (!msg || msg->status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }
    if (msg->cluster != ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT ||
        msg->attribute.id != ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID ||
        msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_S16 ||
        msg->attribute.data.value == nullptr) {
        return ESP_OK;
    }

    const int16_t raw_s16 = read_value<int16_t>(msg->attribute.data.value);
    if (raw_s16 == INT16_MIN) {
        return ESP_OK; // 0x8000 = « mesure invalide » en ZCL : le capteur lié n'a pas de valeur à fournir
    }

    ZbLock lock;
    if (!lock) {
        return ESP_OK;
    }
    const float temp_c = zcl_s16_to_temp(raw_s16);
    if (msg->dst_endpoint == EP_MASTER_Z1) {
        set_external_room_temperature(1, temp_c, true);
    } else if (msg->dst_endpoint == EP_ZONE2) {
        set_external_room_temperature(2, temp_c, true);
    }
    return ESP_OK;
}

// ============================================================================
// Réception des commandes d'écriture depuis Home Assistant (ZHA / Zigbee2MQTT)
// ============================================================================

bool ZbBridge::is_identifying() const {
    if (!identify_active_.load()) {
        return false;
    }
    // Comparaison signée sur 32 bits : insensible au rebouclage du compteur de millisecondes
    // (valable car une identification dure au plus 65535 s, très loin des 24 jours de demi-période).
    if (static_cast<int32_t>(identify_deadline_ms_.load() - static_cast<uint32_t>(now_ms())) > 0) {
        return true;
    }
    identify_active_.store(false);
    return false;
}

esp_err_t ZbBridge::handle_attribute_write(const esp_zb_zcl_set_attr_value_message_t* msg) {
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }
    const void* data = msg->attribute.data.value;
    if (!data) {
        return ESP_OK;
    }

    const uint8_t  ep         = msg->info.dst_endpoint;
    const uint16_t cluster_id = msg->info.cluster;
    const uint16_t attr_id    = msg->attribute.id;

    // Cluster Identify (0x0003) : la pile écrit IdentifyTime (secondes) au début et à la fin de
    // l'identification demandée par le coordinateur -> clignotement de la LED du XIAO.
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
        if (attr_id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
            const uint32_t seconds = read_value<uint16_t>(data);
            identify_deadline_ms_.store(static_cast<uint32_t>(now_ms()) + seconds * 1000UL);
            identify_active_.store(seconds > 0);
        }
        return ESP_OK;
    }

    coordinator_reached_.store(true); // Une écriture reçue prouve que la radio fonctionne dans les deux sens

    if (!bus_) {
        return ESP_OK;
    }

    ZbLock lock;
    if (!lock) {
        return ESP_OK;
    }

    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        // 1. Cluster Thermostat (0x0201) — Zones 1 & 2 + Ballon ECS
        if (ep == EP_DHW) {
            handle_dhw_thermostat_write(attr_id, data);
        } else if (ep == EP_MASTER_Z1 || ep == EP_ZONE2) {
            handle_zone_thermostat_write(ep, attr_id, data);
        }
    } else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT &&
               attr_id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) {
        // 2. Cluster AnalogOutput (0x000D) : consignes réduites, sondes externes, consigne ECS
        handle_analog_output_write(ep, data);
    } else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        // 3. Cluster OnOff (0x0006) : Marche/Arrêt ECS (EP6) et antenne RF externe (EP7)
        handle_on_off_write(ep, data);
    }
    return ESP_OK;
}

void ZbBridge::handle_zone_thermostat_write(uint8_t ep, uint16_t attr_id, const void* data) {
    const bool is_z2 = (ep == EP_ZONE2);
    if (!(is_z2 ? caps_.zone2.present : caps_.zone1.present)) {
        return;
    }
    const uint32_t comfort_field = is_z2 ? bsb::fields::Z2_COMFORT_SETPOINT : bsb::fields::Z1_COMFORT_SETPOINT;
    const uint32_t reduced_field = is_z2 ? bsb::fields::Z2_REDUCED_SETPOINT : bsb::fields::Z1_REDUCED_SETPOINT;
    const uint32_t mode_field    = is_z2 ? bsb::fields::Z2_MODE             : bsb::fields::Z1_MODE;
    const char*    zone_name     = is_z2 ? "Zone 2" : "Zone 1";
    const uint16_t cluster       = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;

    switch (attr_id) {
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID: {
            const bool    is_reduced = (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID);
            const int16_t raw_s16    = read_value<int16_t>(data);
            const float   temp_c     = zcl_s16_to_temp(raw_s16);
            if (temp_c < ZONE_SETPOINT_MIN_C || temp_c > ZONE_SETPOINT_MAX_C ||
                !bus_->enqueue_set_temperature(is_reduced ? reduced_field : comfort_field, temp_c)) {
                ESP_LOGW(TAG, "[Zigbee->BSB] %s Consigne %s %.1f C non transmise (hors plage ou file BSB pleine)",
                         zone_name, is_reduced ? "Reduite" : "Confort", temp_c);
                // La prochaine lecture BSB rétablira et ré-émettra la consigne réelle du régulateur
                forget_value(ep, cluster, attr_id);
                break;
            }
            ESP_LOGI(TAG, "[Zigbee->BSB] %s Consigne %s -> %.1f C", zone_name, is_reduced ? "Reduite" : "Confort", temp_c);
            // Le coordinateur connaît déjà la valeur qu'il vient d'écrire : elle est seulement notée dans
            // le cache (pas de rapport d'écho depuis ce callback, l'attribut pouvant ne pas être encore
            // stocké par la pile). La relecture BSB qui suit l'acquittement fera foi.
            remember_value(ep, cluster, attr_id, static_cast<float>(raw_s16));
            if (is_reduced) {
                set_analog_output_attr(ep, temp_c);
            }
            break;
        }

        case ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID: {
            const uint8_t zcl_mode     = read_value<uint8_t>(data);
            uint8_t&      current_mode = is_z2 ? z2_mode_ : z1_mode_;

            uint8_t bsb_mode = current_mode;
            if (!zcl_system_mode_to_bsb_mode(zcl_mode, bsb_mode)) {
                // Mode sans équivalent (rafraîchissement, ventilation...) : rien n'est envoyé à la PAC et
                // l'attribut revient sur le régime réel, sans quoi Home Assistant afficherait le mode refusé.
                ESP_LOGW(TAG, "[Zigbee->BSB] %s Mode ZCL=0x%02X non pris en charge : ignore", zone_name, zcl_mode);
                set_thermostat_system_mode(ep, bsb_mode_to_zcl_system_mode(current_mode), true);
                schedule_republish(ep, cluster, attr_id);
                break;
            }

            // Toujours confiée au bus, même si elle redemande le régime en cours : c'est lui qui, d'après sa
            // dernière lecture du régulateur, s'épargne l'écriture inutile. Trancher ici d'après le régime
            // mémorisé ferait perdre une commande répétée après un échec d'écriture (bus coupé un instant).
            if (!bus_->enqueue_set_uint8(mode_field, bsb_mode)) {
                ESP_LOGW(TAG, "[Zigbee->BSB] %s Mode non transmis (file BSB pleine)", zone_name);
                bsb_mode = current_mode; // L'attribut ci-dessous revient sur l'état réellement connu
            } else {
                current_mode = bsb_mode;
                ESP_LOGI(TAG, "[Zigbee->BSB] %s Mode -> ZCL=0x%02X (BSB=%u)", zone_name, zcl_mode, bsb_mode);
            }

            // L'attribut reflète le mode réellement demandé à la PAC, pas la valeur brute écrite
            const uint8_t applied_zcl = bsb_mode_to_zcl_system_mode(bsb_mode);
            set_thermostat_system_mode(ep, applied_zcl, true);
            if (applied_zcl != zcl_mode) {
                schedule_republish(ep, cluster, attr_id);
            }
            update_thermostat_running_state();
            break;
        }

        case ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID:
            restore_setpoint_limit(ep, attr_id, ZONE_SETPOINT_MIN_C, ZONE_SETPOINT_MAX_C);
            break;

        default:
            break;
    }
}

void ZbBridge::restore_setpoint_limit(uint8_t ep, uint16_t attr_id, float min_c, float max_c) {
    // Les bornes de consigne sont celles que la passerelle accepte d'écrire sur le régulateur : elles ne
    // sont pas réglables. Une borne modifiée par le coordinateur ferait refuser par la pile des consignes
    // pourtant valides, et tronquerait l'affichage des consignes relues sur le bus. Elle est donc rétablie.
    int16_t fixed = temp_to_zcl_s16(attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID ? min_c : max_c);
    ESP_LOGW(TAG, "[Zigbee] EP%u : les limites de consigne ne sont pas reglables, valeur d'origine retablie (%.1f C)",
             ep, static_cast<double>(fixed) / 100.0);
    write_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id, &fixed);
    schedule_republish(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
}

void ZbBridge::request_dhw_mode(uint8_t target) {
    const uint8_t mode_off = static_cast<uint8_t>(bsb::DhwMode::Off);

    // Toujours confiée au bus, qui s'épargne l'écriture si le régulateur a déjà ce régime (voir les zones)
    if (bus_->enqueue_set_uint8(bsb::fields::DHW_MODE, target)) {
        ESP_LOGI(TAG, "[Zigbee->BSB] Mode Ballon ECS -> %s (BSB=%u)",
                 target == mode_off ? "Arret" : (target == static_cast<uint8_t>(bsb::DhwMode::Eco) ? "Eco" : "Marche"),
                 target);
        dhw_mode_ = target;
    } else {
        ESP_LOGW(TAG, "[Zigbee->BSB] Mode Ballon ECS non transmis (file BSB pleine)");
        target = dhw_mode_; // Les attributs ci-dessous reviennent sur l'état réellement connu
    }

    // Les deux représentations Zigbee de l'ECS (Thermostat SystemMode et interrupteur OnOff) restent alignées
    set_thermostat_system_mode(EP_DHW, dhw_mode_to_zcl_system_mode(target), true);
    set_on_off_attr(EP_DHW, target != mode_off);
    update_thermostat_running_state();
}

void ZbBridge::handle_dhw_thermostat_write(uint16_t attr_id, const void* data) {
    if (!caps_.has_dhw) {
        return;
    }
    const uint16_t cluster = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;

    switch (attr_id) {
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID: {
            const bool    is_reduced = (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID);
            const int16_t raw_s16    = read_value<int16_t>(data);
            const float   temp_c     = zcl_s16_to_temp(raw_s16);
            const uint32_t field     = is_reduced ? bsb::fields::DHW_REDUCED_SETPOINT : bsb::fields::DHW_NOMINAL_SETPOINT;
            if (is_reduced && !caps_.has_dhw_reduced_setpoint) {
                // Prog 1612 absent sur ce régulateur : il n'est ni scruté ni relu, l'écriture ne pourrait
                // jamais être confirmée. Elle est refusée et la valeur d'origine est rétablie côté Zigbee.
                ESP_LOGW(TAG, "[Zigbee->BSB] Consigne reduite ECS (1612) indisponible sur ce regulateur : ecriture ignoree");
                dhw_reduced_restore_pending_ = true; // Republiée par loop_housekeeping(), hors de ce callback
                break;
            }
            if (temp_c < DHW_SETPOINT_MIN_C || temp_c > DHW_SETPOINT_MAX_C ||
                !bus_->enqueue_set_temperature(field, temp_c)) {
                ESP_LOGW(TAG, "[Zigbee->BSB] Ballon ECS Consigne %s %.1f C non transmise (hors plage ou file BSB pleine)",
                         is_reduced ? "Reduite" : "Nominale", temp_c);
                forget_value(EP_DHW, cluster, attr_id);
                break;
            }
            ESP_LOGI(TAG, "[Zigbee->BSB] Ballon ECS Consigne %s (Thermostat) -> %.1f C",
                     is_reduced ? "Reduite" : "Nominale", temp_c);
            remember_value(EP_DHW, cluster, attr_id, static_cast<float>(raw_s16)); // Pas d'écho (cf. zones)
            if (!is_reduced) {
                set_analog_output_attr(EP_DHW, temp_c);
            }
            break;
        }

        case ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID: {
            const uint8_t zcl_mode = read_value<uint8_t>(data);
            // La valeur brute écrite n'est pas forcément celle que nous publions
            forget_value(EP_DHW, cluster, attr_id);
            // Même contrat que les zones : Off = Arrêt, Sleep = Éco, Heat = Marche. « Auto » n'existe pas
            // pour l'ECS (son programme horaire est interne au régulateur) : il vaut simplement « Marche ».
            switch (zcl_mode) {
                case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF:
                    request_dhw_mode(static_cast<uint8_t>(bsb::DhwMode::Off));
                    break;
                case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP:
                    request_dhw_mode(static_cast<uint8_t>(bsb::DhwMode::Eco));
                    break;
                case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT:
                case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_EMERGENCY_HEATING:
                case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO:
                    request_dhw_mode(static_cast<uint8_t>(bsb::DhwMode::On));
                    break;
                default:
                    ESP_LOGW(TAG, "[Zigbee->BSB] Ballon ECS Mode ZCL=0x%02X non pris en charge : ignore", zcl_mode);
                    set_thermostat_system_mode(EP_DHW, dhw_mode_to_zcl_system_mode(dhw_mode_), true);
                    break;
            }
            if (dhw_mode_to_zcl_system_mode(dhw_mode_) != zcl_mode) {
                // Mode retenu différent du mode écrit (« Auto » ramené à Marche, mode refusé...)
                schedule_republish(EP_DHW, cluster, attr_id);
            }
            break;
        }

        case ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID:
        case ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID:
            restore_setpoint_limit(EP_DHW, attr_id, DHW_SETPOINT_MIN_C, DHW_SETPOINT_MAX_C);
            break;

        default:
            break;
    }
}

void ZbBridge::handle_analog_output_write(uint8_t ep, const void* data) {
    const uint16_t cluster = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
    const uint16_t attr    = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID;
    const float    val     = read_value<float>(data);

    // EP 3 / EP 4 : Sonde de température d'ambiance externe (Zone 1 / Zone 2)
    if (ep == EP_HYDRAULIC1 || ep == EP_HYDRAULIC2) {
        // Le coordinateur vient d'écrire cette valeur ; set_external_room_temperature() publie
        // ensuite la valeur effectivement retenue (0 si désactivée, inchangée si hors plage).
        remember_value(ep, cluster, attr, std::isfinite(val) ? val : 0.0f);
        set_external_room_temperature(ep == EP_HYDRAULIC2 ? 2 : 1, val, false);
        {
            const ExternalSensorState& state = (ep == EP_HYDRAULIC2) ? ext_sensor_z2_ : ext_sensor_z1_;
            const float retained = state.active ? state.temperature_c : 0.0f;
            if (!std::isfinite(val) || std::fabs(retained - val) > 0.05f) {
                // Valeur refusée ou ramenée à 0 (désactivation) : à republier après la réponse d'écriture
                schedule_republish(ep, cluster, attr);
            }
        }
        return;
    }

    // EP 1 / EP 2 : Consigne Réduite Zone 1 / 2 (712 / 1012) — EP 6 : Consigne Nominale ECS (1610)
    uint32_t    field       = 0;
    float       min_c       = ZONE_SETPOINT_MIN_C;
    float       max_c       = ZONE_SETPOINT_MAX_C;
    uint16_t    mirror_attr = ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID;
    const char* label       = "";
    if (ep == EP_MASTER_Z1 && caps_.zone1.present) {
        field = bsb::fields::Z1_REDUCED_SETPOINT;
        label = "Zone 1 Consigne Reduite";
    } else if (ep == EP_ZONE2 && caps_.zone2.present) {
        field = bsb::fields::Z2_REDUCED_SETPOINT;
        label = "Zone 2 Consigne Reduite";
    } else if (ep == EP_DHW && caps_.has_dhw) {
        field       = bsb::fields::DHW_NOMINAL_SETPOINT;
        min_c       = DHW_SETPOINT_MIN_C;
        max_c       = DHW_SETPOINT_MAX_C;
        mirror_attr = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID;
        label       = "Consigne Ballon ECS";
    } else {
        return;
    }

    if (!std::isfinite(val) || val < min_c || val > max_c || !bus_->enqueue_set_temperature(field, val)) {
        ESP_LOGW(TAG, "[Zigbee->BSB] %s non transmise (valeur hors plage ou file BSB pleine)", label);
        // La prochaine lecture BSB rétablira et ré-émettra la consigne réelle du régulateur
        forget_value(ep, cluster, attr);
        return;
    }

    ESP_LOGI(TAG, "[Zigbee->BSB] %s (Slider) -> %.1f C", label, val);
    remember_value(ep, cluster, attr, val);
    set_thermostat_temp_attr(ep, mirror_attr, val);
}

void ZbBridge::handle_on_off_write(uint8_t ep, const void* data) {
    const bool on = (read_value<uint8_t>(data) != 0);
    remember_value(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, on ? 1.0f : 0.0f);

    if (ep == EP_DHW && caps_.has_dhw) {
        // EP 6 : Activation Marche/Arrêt Ballon ECS (1600). « Marche » demandée alors que l'ECS est déjà
        // active (Marche ou Éco) : le régime en place est conservé, l'interrupteur ne choisit pas entre les deux.
        const uint8_t mode_off = static_cast<uint8_t>(bsb::DhwMode::Off);
        uint8_t target = mode_off;
        if (on) {
            target = (dhw_mode_known_ && dhw_mode_ != mode_off) ? dhw_mode_ : static_cast<uint8_t>(bsb::DhwMode::On);
        }
        request_dhw_mode(target);
    } else if (ep == EP_BACKUP_AUX) {
        // EP 7 : Sélection Antenne RF Zigbee Externe U.FL (OFF par défaut = Céramique interne)
        set_external_antenna_enabled(on);
    }
}

// ============================================================================
// État de chauffe réel des thermostats (Running State & PI Heating Demand)
// ============================================================================

void ZbBridge::sync_thermostat_action(uint8_t ep, const char* label, const bsb::ThermostatAction& computed,
                                      uint64_t t_now, HeatingPublication& pub) {
    // Une mise en chauffe n'est publiée qu'après confirmation ; l'arrêt, lui, est immédiat
    const bool was_pending = pub.filter.pending();
    const bsb::ThermostatAction action = pub.filter.update(computed, t_now);
    if (was_pending && !computed.heating) {
        ESP_LOGI(TAG, "[%s] Mise en chauffe non confirmee (moins de %lu s) : rien n'est publie "
                      "(PAC=%u, Z1=%u, Z2=%u, ECS=%u)",
                 label, static_cast<unsigned long>(bsb::HEATING_CONFIRM_MS / 1000UL),
                 hp_status_code_, z1_status_code_, z2_status_code_, dhw_status_code_);
    }

    if (pub.last_heating != static_cast<int8_t>(action.heating)) {
        ESP_LOGI(TAG, "[%s] Action CVC -> %s (Demande %u %%, Comp=%s %u%%, Appoint=%s %u%%, PAC=%u, Z1=%u, Z2=%u, ECS=%u)",
                 label, action.heating ? "CHAUFFE" : "INACTIF", action.demand,
                 comp_running_ ? "ON" : "OFF", comp_mod_,
                 backup_running_ ? "ON" : "OFF", backup_mod_,
                 hp_status_code_, z1_status_code_, z2_status_code_, dhw_status_code_);
    }
    pub.last_heating = static_cast<int8_t>(action.heating);
    pub.last_demand  = static_cast<int16_t>(action.demand);

    // ZHA déduit l'action CVC de PIHeatingDemand (> 0 = chauffe) : les deux attributs restent cohérents.
    // La déduplication n'émet que sur transition, avec un heartbeat d'une minute.
    set_thermostat_u16_attr(ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
                            action.heating ? 0x0001 : 0x0000, STATE_HEARTBEAT_MS);
    // Tant que SystemMode vaut Off, la pile refuse d'écrire PIHeatingDemand : il a été remis à 0 au passage
    // en Off (set_thermostat_system_mode) et n'est plus touché. Une chauffe hors-gel en mode Off n'apparaît
    // donc que dans RunningState.
    if (tracked_system_mode(ep) != ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF) {
        set_thermostat_u8_attr(ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID,
                               action.demand, STATE_HEARTBEAT_MS);
    }
}

/**
 * Signale au journal, une fois par épisode, un code statut de zone absent des tables de référence, et ce qui
 * en tient lieu pour décider de la chauffe. Sans cette trace, un thermostat qui reste inactif (ou qui suit
 * la pompe du circuit) à cause d'un code inconnu ne se distingue pas d'un fonctionnement normal.
 */
void ZbBridge::note_zone_evidence(const char* label, uint16_t status, bsb::ZoneEvidence evidence,
                                  uint16_t& logged_code) {
    const bool unlisted = (status != 0) &&
                          (evidence == bsb::ZoneEvidence::Pump || evidence == bsb::ZoneEvidence::None);
    if (!unlisted) {
        logged_code = 0;
        return;
    }
    if (logged_code == status) {
        return;
    }
    logged_code = status;
    if (evidence == bsb::ZoneEvidence::Pump) {
        ESP_LOGW(TAG, "[%s] Code statut %u absent des tables de reference : la demande de chaleur de la zone est "
                      "deduite de la pompe du circuit.", label, status);
    } else {
        ESP_LOGW(TAG, "[%s] Code statut %u absent des tables de reference et aucune pompe de circuit a observer : "
                      "la zone reste affichee inactive.", label, status);
    }
}

void ZbBridge::forget_heat_pump_state() {
    comp_running_     = false;
    comp_mod_         = 0;
    backup_running_   = false;
    backup_mod_       = 0;
    hp_status_code_   = 0;
    hp_pump_running_  = false;
    z1_pump_running_  = false;
    z2_pump_running_  = false;
    z1_status_code_   = 0;
    z2_status_code_   = 0;
    dhw_status_code_  = 0;
    hp_flow_ms_       = 0;
    hp_return_ms_     = 0;
    comp_state_known_ = false;
    comp_mod_known_   = false;
    z1_status_known_  = false;
    z2_status_known_  = false;
    dhw_status_known_ = false;
    // Les modes devront être relus avant qu'une commande « Marche » puisse conserver le régime en place
    z1_mode_known_    = false;
    z2_mode_known_    = false;
    dhw_mode_known_   = false;
}

void ZbBridge::update_thermostat_running_state() {
    bsb::HeatingInputs in;
    in.has_zone1            = caps_.zone1.present;
    in.has_zone2            = caps_.zone2.present;
    in.has_dhw              = caps_.has_dhw;
    in.has_compressor_state = caps_.has_compressor_state;
    in.has_compressor_mod   = caps_.has_compressor_mod;
    in.has_backup_state     = caps_.has_backup_heater;
    in.has_backup_mod       = caps_.has_backup_mod;
    in.zone1_has_pump_state  = caps_.zone1.has_pump_state || caps_.has_hp_pump_state;
    in.zone2_has_pump_state  = caps_.zone2.has_pump_state;
    in.zone1_has_status_code = caps_.zone1.has_status_code;
    in.zone2_has_status_code = caps_.zone2.has_status_code;

    const uint64_t t_now = now_ms();

    // Sondes départ / retour du générateur : utilisées comme garde de plausibilité si elles sont récentes
    in.has_hp_temps = caps_.has_hp_flow_temp && caps_.has_hp_return_temp &&
                      hp_flow_ms_ != 0 && hp_return_ms_ != 0 &&
                      elapsed_ms(t_now, hp_flow_ms_) < BSB_STALE_TIMEOUT_MS &&
                      elapsed_ms(t_now, hp_return_ms_) < BSB_STALE_TIMEOUT_MS;
    in.hp_flow_c   = hp_flow_c_;
    in.hp_return_c = hp_return_c_;

    // Au démarrage du compresseur, l'état (8400) est souvent lu avant sa modulation (8413) : la demande
    // calculée vaut alors 100 % par défaut. Elle n'est pas publiée telle quelle, la mise en chauffe devant
    // d'abord être confirmée (HEATING_CONFIRM_MS), ce qui laisse le temps de relire la modulation réelle.
    in.compressor_on  = comp_running_;
    in.compressor_mod = comp_mod_;
    in.backup_on      = backup_running_;
    in.backup_mod     = backup_mod_;
    in.hp_status      = hp_status_code_;

    in.zone1_status          = z1_status_code_;
    in.zone2_status          = z2_status_code_;
    in.dhw_status            = dhw_status_code_;
    in.zone1_pump_on         = z1_pump_running_ || hp_pump_running_;
    in.zone2_pump_on         = z2_pump_running_;
    in.zone1_protection_mode = (z1_mode_ == static_cast<uint8_t>(bsb::HeatingMode::Protection));
    in.zone2_protection_mode = (z2_mode_ == static_cast<uint8_t>(bsb::HeatingMode::Protection));

    // Fraîcheur des entrées. Ce qui compte n'est pas que « quelqu'un » reçoive des réponses sur le bus
    // (l'autre maître 0x0A en reçoit en permanence), mais que l'état du générateur ait été relu récemment :
    // si nos propres lectures 8400 / 8413 / 8006 / 8402 ne sont plus servies, l'état mémorisé est périmé.
    const bool generator_observable = caps_.has_compressor_state || caps_.has_compressor_mod ||
                                      caps_.has_hp_status_code || caps_.has_backup_heater || caps_.has_backup_mod;
    const uint64_t freshness_ms = generator_observable ? last_generator_ms_ : last_bsb_value_ms_;
    if (freshness_ms == 0 && t_now < BSB_STALE_TIMEOUT_MS) {
        // Juste après le démarrage, rien n'a encore été lu : publier « inactif » maintenant ferait voir à
        // Home Assistant un arrêt puis une reprise de chauffe à chaque redémarrage. On attend la première
        // lecture ; si elle ne vient pas dans le délai de péremption, « inactif » sera publié comme prévu.
        return;
    }
    const bool bsb_alive = (freshness_ms != 0) && elapsed_ms(t_now, freshness_ms) < BSB_STALE_TIMEOUT_MS;
    if (bsb_alive != bsb_alive_) {
        bsb_alive_ = bsb_alive;
        if (bsb_alive) {
            ESP_LOGI(TAG, "Etat de la PAC de nouveau lu sur le bus BSB : reprise du suivi de l'etat de chauffe.");
        } else {
            ESP_LOGW(TAG, "Etat de la PAC non relu depuis %lu min (bus BSB muet) : thermostats remis a inactif.",
                     static_cast<unsigned long>(BSB_STALE_TIMEOUT_MS / 60000ULL));
            // L'état mémorisé est périmé : à la reprise, chaque entrée devra avoir été relue avant de compter
            forget_heat_pump_state();
        }
    }

    // Même précaution tant que toutes les entrées n'ont pas été lues une première fois : le générateur est
    // souvent connu une à deux secondes avant les codes d'état des zones, et un calcul sur des entrées
    // partielles ferait passer chaque thermostat par « inactif » avant de revenir à « chauffe ».
    const bool inputs_complete =
        (!caps_.has_compressor_state || comp_state_known_) &&
        (!caps_.has_compressor_mod || comp_mod_known_) &&
        (!caps_.zone1.present || !caps_.zone1.has_status_code || z1_status_known_) &&
        (!caps_.zone2.present || !caps_.zone2.has_status_code || z2_status_known_) &&
        (!caps_.has_dhw || !caps_.has_dhw_status_code || dhw_status_known_);
    if (bsb_alive && !inputs_complete && t_now < BSB_STALE_TIMEOUT_MS) {
        return;
    }

    bsb::HeatingOutputs out; // Par défaut : aucun thermostat en chauffe
    if (bsb_alive) {
        out = bsb::compute_heating_state(in);
    }
    // La garde hydraulique retire la chauffe à une zone en demande (pendant une charge ECS, un retour plus
    // chaud que le départ ne change rien). C'est l'état normal d'une mise en route, ou d'une reprise du
    // chauffage à la fin d'une charge ECS, le temps que l'eau chauffée arrive : seul un état qui dure est une
    // anomalie, signalée une fois par épisode.
    const bool was_refuted = unfed_watch_.in_grace_period() || unfed_watch_.alerted();
    if (unfed_watch_.update(out.zone_heating_refuted, t_now)) {
        ESP_LOGW(TAG, "Le regulateur annonce le generateur en marche pour le chauffage depuis %lu min mais le retour "
                      "(%.1f C) reste plus chaud que le depart (%.1f C) : aucune chaleur n'atteint les zones, elles "
                      "restent affichees inactives.",
                 static_cast<unsigned long>(bsb::UNFED_ZONE_ALERT_MS / 60000UL), hp_return_c_, hp_flow_c_);
    } else if (out.zone_heating_refuted && !was_refuted) {
        ESP_LOGI(TAG, "Generateur annonce en marche, mais le depart (%.1f C) est encore plus froid que le retour "
                      "(%.1f C) : la zone en demande sera affichee en chauffe quand la chaleur arrivera.",
                 hp_flow_c_, hp_return_c_);
    }

    if (caps_.zone1.present) {
        note_zone_evidence("Chauffage Z1", z1_status_code_, out.zone1_evidence, z1_unlisted_logged_);
        sync_thermostat_action(EP_MASTER_Z1, "Chauffage Z1", out.zone1, t_now, z1_heating_);
    }
    if (caps_.zone2.present) {
        note_zone_evidence("Chauffage Z2", z2_status_code_, out.zone2_evidence, z2_unlisted_logged_);
        sync_thermostat_action(EP_ZONE2, "Chauffage Z2", out.zone2, t_now, z2_heating_);
    }
    if (caps_.has_dhw) {
        sync_thermostat_action(EP_DHW, "Ballon ECS", out.dhw, t_now, dhw_heating_);
    }
}

// ============================================================================
// Mesures que le régulateur ne fournit plus -> attributs Zigbee « indisponibles »
// ============================================================================

void ZbBridge::track_measurements(uint64_t t_now) {
    using namespace bsb::fields;
    // Mêmes conditions que la table de scrutation (BsbDiscovery::register_discovered_polls) : ne sont
    // surveillés que les paramètres réellement relus, et qui alimentent un attribut exposé.
    const auto track = [&](bool wanted, uint32_t field_id) {
        if (wanted && !watch_.track(field_id, t_now)) {
            ESP_LOGW(TAG, "Suivi de fraicheur plein : %s ne sera pas surveille.", bsb::get_field_name(field_id));
        }
    };
    track(caps_.has_outdoor_temp,     OUTDOOR_TEMP);
    track(caps_.has_hp_flow_temp,     HP_FLOW_TEMP);
    track(caps_.has_hp_return_temp,   HP_RETURN_TEMP);
    track(caps_.has_compressor_state, COMPRESSOR_STATE);
    track(caps_.has_compressor_mod,   COMPRESSOR_MOD);
    track(caps_.has_hp_pump_state,    HP_PUMP_STATE);
    track(caps_.has_backup_heater,    BACKUP_HEATER_STATE);
    track(caps_.has_backup_mod,       BACKUP_MOD);
    track(caps_.has_hp_status_code,   HP_STATUS_CODE);

    track(caps_.zone1.present && caps_.zone1.has_status_code, Z1_STATUS_CODE);
    track(caps_.zone1.present && caps_.zone1.has_pump_state,  Z1_PUMP_STATE);
    track(caps_.zone1.present && caps_.zone1.has_room_sensor, Z1_ROOM_TEMP);
    track(caps_.zone1.present && caps_.zone1.has_flow_sensor, Z1_FLOW_TEMP);
    track(caps_.zone1.present,                                Z1_ACTIVE_SETPOINT);

    track(caps_.zone2.present && caps_.zone2.has_status_code, Z2_STATUS_CODE);
    track(caps_.zone2.present && caps_.zone2.has_pump_state,  Z2_PUMP_STATE);
    track(caps_.zone2.present && caps_.zone2.has_room_sensor, Z2_ROOM_TEMP);
    track(caps_.zone2.present && caps_.zone2.has_flow_sensor, Z2_FLOW_TEMP);
    track(caps_.zone2.present,                                Z2_ACTIVE_SETPOINT);

    track(caps_.has_dhw && caps_.has_dhw_tank_temp, DHW_TANK_TEMP);
    track(caps_.has_dhw,                            DHW_STATUS_CODE);
}

/**
 * Publie « indisponible » sur les attributs qu'alimente un paramètre BSB : valeur invalide ZCL (0x8000)
 * pour une température, NaN pour une grandeur analogique. Un état binaire n'a pas de valeur « inconnue » :
 * il repasse à l'arrêt, comme les thermostats repassent à « inactif » — rien n'est affirmé sans lecture.
 * Les conditions sont celles de on_bsb_value() : chaque attribut est rendu indisponible par le paramètre
 * qui l'alimente en temps normal, et par lui seul.
 */
void ZbBridge::publish_unavailable(uint32_t field_id) {
    using namespace bsb::fields;
    const float    none       = std::numeric_limits<float>::quiet_NaN();
    const uint16_t local_temp = ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID;
    const uint16_t outdoor    = ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID;

    switch (field_id) {
        // --- Capteurs généraux & générateur ---
        case OUTDOOR_TEMP:
            if (caps_.has_outdoor_temp) set_temp_meas_attr(EP_MASTER_Z1, none);
            if (caps_.zone1.present)    set_thermostat_temp_attr(EP_MASTER_Z1, outdoor, none);
            if (caps_.zone2.present)    set_thermostat_temp_attr(EP_ZONE2, outdoor, none);
            break;
        case HP_FLOW_TEMP:
            if (caps_.has_hp_flow_temp) set_temp_meas_attr(EP_HYDRAULIC1, none);
            break;
        case HP_RETURN_TEMP:
            if (caps_.has_hp_return_temp) set_temp_meas_attr(EP_HYDRAULIC2, none);
            break;
        case COMPRESSOR_STATE:
            if (caps_.has_compressor_state) set_binary_input_attr(EP_MASTER_Z1, false);
            break;
        case COMPRESSOR_MOD:
            if (caps_.has_compressor_mod) set_analog_input_attr(EP_MASTER_Z1, none);
            break;
        case HP_PUMP_STATE:
            if (caps_.has_hp_pump_state && has_ep_hydraulic1()) set_binary_input_attr(EP_HYDRAULIC1, false);
            break;
        case BACKUP_HEATER_STATE:
            if (caps_.has_backup_heater) set_binary_input_attr(EP_BACKUP_AUX, false);
            break;
        case BACKUP_MOD:
            if (caps_.has_backup_mod) set_analog_input_attr(EP_BACKUP_AUX, none);
            break;
        case HP_STATUS_CODE:
            if (caps_.has_hp_status_code && has_ep_hydraulic1()) {
                set_analog_input_attr(EP_HYDRAULIC1, none, STATE_HEARTBEAT_MS);
            }
            break;

        // --- Zone 1 ---
        case Z1_STATUS_CODE:
            if (caps_.zone1.has_status_code) set_analog_input_attr(EP_HYDRAULIC2, none, STATE_HEARTBEAT_MS);
            break;
        case Z1_PUMP_STATE:
            if (!caps_.has_hp_pump_state && caps_.zone1.has_pump_state && has_ep_hydraulic1()) {
                set_binary_input_attr(EP_HYDRAULIC1, false);
            }
            break;
        case Z1_ROOM_TEMP:
            if (caps_.zone1.present && caps_.zone1.has_room_sensor) {
                set_temp_meas_attr(EP_BACKUP_AUX, none);
                if (!ext_sensor_z1_.active) set_thermostat_temp_attr(EP_MASTER_Z1, local_temp, none);
            }
            break;
        case Z1_FLOW_TEMP:
            if (caps_.zone1.has_flow_sensor && !caps_.zone2.has_flow_sensor) set_temp_meas_attr(EP_FLOW_Z2, none);
            break;
        case Z1_ACTIVE_SETPOINT:
            if (caps_.zone1.present && !caps_.zone1.has_room_sensor && !ext_sensor_z1_.active) {
                set_thermostat_temp_attr(EP_MASTER_Z1, local_temp, none);
            }
            break;

        // --- Zone 2 ---
        case Z2_STATUS_CODE:
            if (caps_.zone2.present && caps_.zone2.has_status_code) {
                set_analog_input_attr(EP_ZONE2, none, STATE_HEARTBEAT_MS);
            }
            break;
        case Z2_PUMP_STATE:
            if (caps_.zone2.present && caps_.zone2.has_pump_state) set_binary_input_attr(EP_ZONE2, false);
            break;
        case Z2_ROOM_TEMP:
            if (caps_.zone2.present && caps_.zone2.has_room_sensor) {
                set_temp_meas_attr(EP_ZONE2, none);
                if (!ext_sensor_z2_.active) set_thermostat_temp_attr(EP_ZONE2, local_temp, none);
            }
            break;
        case Z2_FLOW_TEMP:
            if (caps_.zone2.has_flow_sensor) {
                set_temp_meas_attr(EP_FLOW_Z2, none);
                if (caps_.zone2.present && !caps_.zone2.has_room_sensor) set_temp_meas_attr(EP_ZONE2, none);
            }
            break;
        case Z2_ACTIVE_SETPOINT:
            if (caps_.zone2.present) {
                if (has_ep_flow()) set_analog_input_attr(EP_FLOW_Z2, none);
                if (!caps_.zone2.has_room_sensor && !ext_sensor_z2_.active) {
                    set_thermostat_temp_attr(EP_ZONE2, local_temp, none);
                }
            }
            break;

        // --- Ballon ECS ---
        case DHW_TANK_TEMP:
            if (caps_.has_dhw) {
                if (caps_.has_dhw_tank_temp) set_temp_meas_attr(EP_DHW, none);
                set_thermostat_temp_attr(EP_DHW, local_temp, none);
            }
            break;
        case DHW_STATUS_CODE:
            if (caps_.has_dhw) set_analog_input_attr(EP_DHW, none, STATE_HEARTBEAT_MS);
            break;

        default:
            break;
    }
}

void ZbBridge::expire_stale_measurements(uint64_t t_now) {
    uint32_t     expired[bsb::MeasurementWatch::MAX_FIELDS];
    const size_t count = watch_.expire(t_now, BSB_STALE_TIMEOUT_MS, expired, bsb::MeasurementWatch::MAX_FIELDS);
    if (count == 0) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        publish_unavailable(expired[i]);
    }
    ESP_LOGW(TAG, "%u mesure(s) non relue(s) depuis %lu min (%s%s) : affichee(s) indisponible(s) dans Zigbee "
                  "plutot que figee(s) sur leur derniere valeur.",
             static_cast<unsigned>(count), static_cast<unsigned long>(BSB_STALE_TIMEOUT_MS / 60000ULL),
             bsb::get_field_name(expired[0]), count > 1 ? ", ..." : "");
}

// ============================================================================
// Nouvelle valeur lue sur le bus BSB -> attributs Zigbee
// ============================================================================

void ZbBridge::on_bsb_value(const bsb::DecodedValue& val) {
    if (!s_stack_running.load()) return;

    ZbLock lock;
    if (!lock) {
        // Pile Zigbee indisponible : la valeur sera relue au prochain cycle de scrutation
        zb_lock_timeouts_++;
        return;
    }

    if (!val.valid) {
        // Le régulateur a répondu, mais sans valeur exploitable (« --- » d'une sonde en défaut, mesure hors
        // plage) : la mesure exposée dans Zigbee ne reste pas figée sur son ancienne valeur.
        if (watch_.note_no_value(val.field_id)) {
            ESP_LOGW(TAG, "%s : le regulateur ne fournit plus de valeur (\"---\"), mesure affichee indisponible.",
                     bsb::get_field_name(val.field_id));
            publish_unavailable(val.field_id);
        }
        return;
    }
    last_bsb_value_ms_ = now_ms(); // Le régulateur répond : l'état mémorisé de la PAC est à jour
    if (watch_.note_value(val.field_id, last_bsb_value_ms_)) {
        ESP_LOGI(TAG, "%s : mesure de nouveau fournie par le regulateur.", bsb::get_field_name(val.field_id));
    }

    using namespace bsb::fields;
    const uint16_t thermostat_local_temp = ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID;

    switch (val.field_id) {
        // --- Capteurs généraux & Hydrauliques PAC ---
        case OUTDOOR_TEMP:
            if (val.float_val < -50.0f || val.float_val > 70.0f) {
                break;
            }
            if (caps_.has_outdoor_temp) {
                set_temp_meas_attr(EP_MASTER_Z1, val.float_val);
            }
            if (caps_.zone1.present) {
                set_thermostat_temp_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, val.float_val);
            }
            if (caps_.zone2.present) {
                set_thermostat_temp_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, val.float_val);
            }
            break;

        case HP_FLOW_TEMP:
            hp_flow_c_  = val.float_val;
            hp_flow_ms_ = now_ms();
            if (caps_.has_hp_flow_temp) {
                set_temp_meas_attr(EP_HYDRAULIC1, val.float_val);
            }
            update_thermostat_running_state(); // L'écart départ / retour entre dans l'état de chauffe
            break;

        case HP_RETURN_TEMP:
            hp_return_c_  = val.float_val;
            hp_return_ms_ = now_ms();
            if (caps_.has_hp_return_temp) {
                set_temp_meas_attr(EP_HYDRAULIC2, val.float_val);
            }
            update_thermostat_running_state();
            break;

        case COMPRESSOR_STATE:
            if (val.flag_byte == 0x00) {
                const bool was_running = comp_running_;
                last_generator_ms_ = now_ms();
                comp_running_      = val.bool_val;
                comp_state_known_  = true;
                if (caps_.has_compressor_state) {
                    set_binary_input_attr(EP_MASTER_Z1, comp_running_);
                }
                if (comp_running_ && !was_running && caps_.has_compressor_mod && bus_) {
                    bus_->request_refresh(COMPRESSOR_MOD, 300); // Modulation réelle au plus vite
                }
                if (!comp_running_) {
                    comp_mod_ = 0;
                    if (caps_.has_compressor_mod) {
                        set_analog_input_attr(EP_MASTER_Z1, 0.0f);
                    }
                }
                update_thermostat_running_state();
            }
            break;

        case COMPRESSOR_MOD: {
            // Le décodage ramène déjà la modulation à 0 lorsque le régulateur répond « --- »
            last_generator_ms_ = now_ms();
            comp_mod_       = clamp_percent(val.int_val);
            comp_mod_known_ = true;
            if (caps_.has_compressor_mod) {
                const bool compressor_on = caps_.has_compressor_state ? comp_running_ : (comp_mod_ > 0);
                set_analog_input_attr(EP_MASTER_Z1, compressor_on ? static_cast<float>(comp_mod_) : 0.0f);
            }
            update_thermostat_running_state();
            break;
        }

        case HP_PUMP_STATE:
            // Sur RVS21, 8406 renvoie flag_byte=0x01 ("---") s'il n'y a pas de pompe primaire Q9 dédiée
            if (val.flag_byte == 0x00) {
                hp_pump_running_ = val.bool_val;
                if (caps_.has_hp_pump_state && has_ep_hydraulic1()) {
                    set_binary_input_attr(EP_HYDRAULIC1, val.bool_val);
                }
                update_thermostat_running_state();
            }
            break;

        case BACKUP_HEATER_STATE:
            if (val.flag_byte == 0x00) {
                last_generator_ms_ = now_ms();
                backup_running_ = val.bool_val;
                if (caps_.has_backup_heater) {
                    set_binary_input_attr(EP_BACKUP_AUX, val.bool_val);
                }
                update_thermostat_running_state();
            }
            break;

        case BACKUP_MOD:
            last_generator_ms_ = now_ms();
            backup_mod_ = clamp_percent(val.int_val);
            if (caps_.has_backup_mod) {
                set_analog_input_attr(EP_BACKUP_AUX, static_cast<float>(backup_mod_));
            }
            update_thermostat_running_state();
            break;

        case HP_STATUS_CODE:
            last_generator_ms_ = now_ms();
            hp_status_code_ = clamp_status_code(val.int_val);
            ESP_LOGD(TAG, "Statut PAC (8006) = %u (%s)", hp_status_code_, bsb::get_hp_status_text_fr(hp_status_code_));
            if (caps_.has_hp_status_code && has_ep_hydraulic1()) {
                set_analog_input_attr(EP_HYDRAULIC1, static_cast<float>(hp_status_code_), STATE_HEARTBEAT_MS);
            }
            update_thermostat_running_state();
            break;

        // --- Zone 1 ---
        case Z1_MODE:
            if (caps_.zone1.present && val.int_val >= 0 && val.int_val <= 3) {
                z1_mode_       = static_cast<uint8_t>(val.int_val);
                z1_mode_known_ = true;
                set_thermostat_system_mode(EP_MASTER_Z1, bsb_mode_to_zcl_system_mode(z1_mode_), false);
                update_thermostat_running_state();
            }
            break;

        case Z1_COMFORT_SETPOINT:
            if (caps_.zone1.present) {
                set_thermostat_temp_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                         val.float_val);
            }
            break;

        case Z1_REDUCED_SETPOINT:
            if (caps_.zone1.present) {
                set_thermostat_temp_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID,
                                         val.float_val);
                set_analog_output_attr(EP_MASTER_Z1, val.float_val);
            }
            break;

        case Z1_ACTIVE_SETPOINT:
            if (caps_.zone1.present && !caps_.zone1.has_room_sensor && !ext_sensor_z1_.active) {
                // Sans sonde d'ambiance filaire ni sonde externe active, afficher la consigne active
                set_thermostat_temp_attr(EP_MASTER_Z1, thermostat_local_temp, val.float_val);
            }
            break;

        case Z1_ROOM_TEMP:
            if (caps_.zone1.present) {
                check_external_sensor_feedback(1, val.float_val);
                if (caps_.zone1.has_room_sensor && val.float_val < 45.0f) {
                    set_temp_meas_attr(EP_BACKUP_AUX, val.float_val);
                    if (!ext_sensor_z1_.active) {
                        set_thermostat_temp_attr(EP_MASTER_Z1, thermostat_local_temp, val.float_val);
                    }
                }
            }
            break;

        case Z1_FLOW_TEMP:
            if (caps_.zone1.has_flow_sensor && !caps_.zone2.has_flow_sensor) {
                set_temp_meas_attr(EP_FLOW_Z2, val.float_val);
            }
            break;

        case Z1_PUMP_STATE:
            if (val.flag_byte == 0x00) {
                z1_pump_running_ = val.bool_val;
                // L'entrée binaire de l'EP3 porte la pompe Q2 uniquement en l'absence de circulateur PAC (Q9)
                if (!caps_.has_hp_pump_state && caps_.zone1.has_pump_state && has_ep_hydraulic1()) {
                    set_binary_input_attr(EP_HYDRAULIC1, val.bool_val);
                }
                update_thermostat_running_state();
            }
            break;

        case Z1_STATUS_CODE:
            z1_status_code_  = clamp_status_code(val.int_val);
            z1_status_known_ = true;
            ESP_LOGD(TAG, "Statut Zone 1 (8000) = %u (%s)", z1_status_code_, bsb::get_zone_status_text_fr(z1_status_code_));
            if (caps_.zone1.has_status_code) {
                set_analog_input_attr(EP_HYDRAULIC2, static_cast<float>(z1_status_code_), STATE_HEARTBEAT_MS);
            }
            update_thermostat_running_state();
            break;

        // --- Zone 2 ---
        case Z2_MODE:
            if (caps_.zone2.present && val.int_val >= 0 && val.int_val <= 3) {
                z2_mode_       = static_cast<uint8_t>(val.int_val);
                z2_mode_known_ = true;
                set_thermostat_system_mode(EP_ZONE2, bsb_mode_to_zcl_system_mode(z2_mode_), false);
                update_thermostat_running_state();
            }
            break;

        case Z2_COMFORT_SETPOINT:
            if (caps_.zone2.present) {
                set_thermostat_temp_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                         val.float_val);
            }
            break;

        case Z2_REDUCED_SETPOINT:
            if (caps_.zone2.present) {
                set_thermostat_temp_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID,
                                         val.float_val);
                set_analog_output_attr(EP_ZONE2, val.float_val);
            }
            break;

        case Z2_ACTIVE_SETPOINT:
            if (caps_.zone2.present) {
                if (has_ep_flow()) {
                    set_analog_input_attr(EP_FLOW_Z2, val.float_val);
                }
                if (!caps_.zone2.has_room_sensor && !ext_sensor_z2_.active) {
                    // Même règle que la Zone 1 : sans mesure d'ambiance, afficher la consigne active
                    set_thermostat_temp_attr(EP_ZONE2, thermostat_local_temp, val.float_val);
                }
            }
            break;

        case Z2_ROOM_TEMP:
            if (caps_.zone2.present) {
                check_external_sensor_feedback(2, val.float_val);
                if (caps_.zone2.has_room_sensor && val.float_val < 45.0f) {
                    set_temp_meas_attr(EP_ZONE2, val.float_val);
                    if (!ext_sensor_z2_.active) {
                        set_thermostat_temp_attr(EP_ZONE2, thermostat_local_temp, val.float_val);
                    }
                }
            }
            break;

        case Z2_FLOW_TEMP:
            if (caps_.zone2.has_flow_sensor) {
                set_temp_meas_attr(EP_FLOW_Z2, val.float_val);
                // Sans sonde d'ambiance filaire, le cluster TemperatureMeasurement de l'EP2 est créé
                // pour la température de départ de la zone : il doit donc être alimenté lui aussi.
                if (caps_.zone2.present && !caps_.zone2.has_room_sensor) {
                    set_temp_meas_attr(EP_ZONE2, val.float_val);
                }
            }
            break;

        case Z2_PUMP_STATE:
            if (val.flag_byte == 0x00) {
                z2_pump_running_ = val.bool_val;
                if (caps_.zone2.present && caps_.zone2.has_pump_state) {
                    set_binary_input_attr(EP_ZONE2, val.bool_val);
                }
                update_thermostat_running_state();
            }
            break;

        case Z2_STATUS_CODE:
            z2_status_code_  = clamp_status_code(val.int_val);
            z2_status_known_ = true;
            ESP_LOGD(TAG, "Statut Zone 2 (8001) = %u (%s)", z2_status_code_, bsb::get_zone_status_text_fr(z2_status_code_));
            if (caps_.zone2.present && caps_.zone2.has_status_code) {
                set_analog_input_attr(EP_ZONE2, static_cast<float>(z2_status_code_), STATE_HEARTBEAT_MS);
            }
            update_thermostat_running_state();
            break;

        // --- Ballon ECS (DHW) ---
        case DHW_TANK_TEMP:
            if (caps_.has_dhw) {
                if (caps_.has_dhw_tank_temp) {
                    set_temp_meas_attr(EP_DHW, val.float_val);
                }
                set_thermostat_temp_attr(EP_DHW, thermostat_local_temp, val.float_val);
            }
            break;

        case DHW_NOMINAL_SETPOINT:
            if (caps_.has_dhw) {
                set_analog_output_attr(EP_DHW, val.float_val);
                set_thermostat_temp_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, val.float_val);
            }
            break;

        case DHW_REDUCED_SETPOINT:
            if (caps_.has_dhw) {
                set_thermostat_temp_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, val.float_val);
            }
            break;

        case DHW_MODE:
            if (caps_.has_dhw && val.int_val >= 0 && val.int_val <= 2) {
                dhw_mode_       = static_cast<uint8_t>(val.int_val);
                dhw_mode_known_ = true;
                const bool dhw_on = (dhw_mode_ != static_cast<uint8_t>(bsb::DhwMode::Off));
                set_thermostat_system_mode(EP_DHW, dhw_mode_to_zcl_system_mode(dhw_mode_), false);
                set_on_off_attr(EP_DHW, dhw_on);
            }
            break;

        case DHW_STATUS_CODE:
            dhw_status_code_  = clamp_status_code(val.int_val);
            dhw_status_known_ = true;
            ESP_LOGD(TAG, "Statut ECS (8003) = %u (%s)", dhw_status_code_, bsb::get_dhw_status_text_fr(dhw_status_code_));
            if (caps_.has_dhw) {
                set_analog_input_attr(EP_DHW, static_cast<float>(dhw_status_code_), STATE_HEARTBEAT_MS);
            }
            update_thermostat_running_state();
            break;

        default:
            break;
    }
}

// ============================================================================
// Support Mise à Jour Firmware OTA Zigbee (ZCL 0x0019 + zha_toolkit.ota_notify)
// ============================================================================

static esp_err_t parse_ota_sub_element(uint32_t total_size,
                                       const void* payload,
                                       uint16_t payload_size,
                                       const void** out_buf,
                                       uint16_t* out_len) {
    if (!s_ota_tag_received) {
        if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN) {
            ESP_LOGE(TAG, "[OTA] En-tete de sous-element OTA invalide (taille=%u)", payload_size);
            return ESP_ERR_INVALID_ARG;
        }

        const uint8_t* ptr = static_cast<const uint8_t*>(payload);
        uint16_t tag_id = static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
        uint32_t length = static_cast<uint32_t>(ptr[2]) |
                          (static_cast<uint32_t>(ptr[3]) << 8) |
                          (static_cast<uint32_t>(ptr[4]) << 16) |
                          (static_cast<uint32_t>(ptr[5]) << 24);

        if (total_size < OTA_ELEMENT_HEADER_LEN || length != (total_size - OTA_ELEMENT_HEADER_LEN)) {
            ESP_LOGE(TAG, "[OTA] Longueur sous-element invalide (%lu + 6 != %lu)",
                     static_cast<unsigned long>(length),
                     static_cast<unsigned long>(total_size));
            return ESP_ERR_INVALID_ARG;
        }

        s_ota_tag_id       = tag_id;
        s_ota_tag_received = true;

        *out_buf = ptr + OTA_ELEMENT_HEADER_LEN;
        *out_len = payload_size - OTA_ELEMENT_HEADER_LEN;
    } else {
        *out_buf = payload;
        *out_len = payload_size;
    }

    if (s_ota_tag_id != OTA_UPGRADE_IMAGE_TAG) {
        ESP_LOGE(TAG, "[OTA] Tag de sous-element non supporte : 0x%04X", s_ota_tag_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void find_ota_server_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void* user_ctx) {
    (void)user_ctx;
    uint16_t srv_addr = (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) ? addr : 0x0000;
    uint8_t  srv_ep   = (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) ? endpoint : 1;
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ZbBridge::instance().note_coordinator_reached(); // Le coordinateur a répondu à notre requête
    }

    esp_zb_ota_upgrade_client_query_interval_set(ZbBridge::EP_MASTER_Z1, ZB_BSB_OTA_QUERY_INTERVAL_MIN);
    esp_zb_ota_upgrade_client_query_image_req(srv_addr, srv_ep);
    ESP_LOGI(TAG, "[OTA] Client OTA actif (Serveur=0x%04X EP=%u, Version courante=0x%08lX, Manuf=0x%04X, ImageType=0x%04X)",
             srv_addr, srv_ep,
             static_cast<unsigned long>(ZB_BSB_OTA_FILE_VERSION),
             ZB_BSB_OTA_MANUFACTURER_CODE,
             ZB_BSB_OTA_IMAGE_TYPE);
}

void ZbBridge::request_ota_update() {
    static uint16_t ota_cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
    esp_zb_zdo_match_desc_req_param_t req = {};
    req.addr_of_interest = 0x0000;
    req.dst_nwk_addr     = 0x0000;
    req.num_in_clusters  = 1;
    req.num_out_clusters = 0;
    req.profile_id       = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_list     = ota_cluster_list;

    if (esp_zb_bdb_dev_joined()) {
        esp_zb_zdo_match_cluster(&req, find_ota_server_cb, nullptr);
    }
}

esp_err_t ZbBridge::handle_ota_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* msg) {
    if (!msg) return ESP_OK;
    if (msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        coordinator_reached_.store(true); // Réponse du serveur OTA reçue
    }

    // info.status ne dit que si la réponse a été reçue ; c'est query_status qui porte la réponse du
    // serveur (0x98 « pas d'image disponible » lors de chaque interrogation périodique sans mise à jour).
    if (msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS || msg->query_status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGD(TAG, "[OTA] Aucune mise a jour proposee par le serveur (statuts ZCL 0x%02X / 0x%02X)",
                 msg->info.status, msg->query_status);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "[OTA] Image disponible sur 0x%04X EP %u -> Version=0x%08lX (actuelle=0x%08lX), Manuf=0x%04X, Type=0x%04X, Taille=%lu octets",
             msg->server_addr.u.short_addr,
             msg->server_endpoint,
             static_cast<unsigned long>(msg->file_version),
             static_cast<unsigned long>(ZB_BSB_OTA_FILE_VERSION),
             msg->manufacturer_code,
             msg->image_type,
             static_cast<unsigned long>(msg->image_size));

    if (msg->image_size == 0 || msg->file_version == 0) {
        ESP_LOGW(TAG, "[OTA] Image rejetee (taille ou version nulle)");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[OTA] Validation de l'image OTA -> Demarrage du telechargement...");
    return ESP_OK;
}

esp_err_t ZbBridge::handle_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* msg) {
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    s_ota_last_activity_ms = now_ms();

    switch (msg->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START: {
            ESP_LOGI(TAG, "[OTA] === Demarrage de la mise a jour firmware Zigbee OTA ===");
            // Un transfert précédent interrompu sans notification ne doit pas laisser de session ouverte
            ota_abort_transfer();
            ota_in_progress_.store(true);
            s_ota_last_pct_log  = 255;
            s_ota_start_time_us = esp_timer_get_time();

            s_ota_partition = esp_ota_get_next_update_partition(nullptr);
            if (!s_ota_partition) {
                ESP_LOGE(TAG, "[OTA] Aucune partition OTA disponible !");
                ota_in_progress_.store(false);
                return ESP_FAIL;
            }

            ESP_LOGI(TAG, "[OTA] Ecriture vers la partition '%s' (offset=0x%08lX, taille=%lu)",
                     s_ota_partition->label,
                     static_cast<unsigned long>(s_ota_partition->address),
                     static_cast<unsigned long>(s_ota_partition->size));

            ret = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "[OTA] Echec esp_ota_begin : %s", esp_err_to_name(ret));
                s_ota_handle = 0;
                ota_in_progress_.store(false);
                return ret;
            }
            break;
        }

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE: {
            if (!s_ota_handle || !s_ota_partition) {
                ESP_LOGE(TAG, "[OTA] Bloc recu sans session OTA ouverte");
                ota_in_progress_.store(false);
                return ESP_ERR_INVALID_STATE;
            }

            s_ota_total_size = msg->ota_header.image_size;
            // L'image (hors en-tête de sous-élément) doit tenir dans la partition applicative de destination
            if (s_ota_total_size > (s_ota_partition->size + OTA_ELEMENT_HEADER_LEN)) {
                ESP_LOGE(TAG, "[OTA] Image trop volumineuse (%lu octets) pour la partition '%s' (%lu octets)",
                         static_cast<unsigned long>(s_ota_total_size), s_ota_partition->label,
                         static_cast<unsigned long>(s_ota_partition->size));
                ota_abort_transfer();
                ota_in_progress_.store(false);
                return ESP_ERR_INVALID_SIZE;
            }
            s_ota_offset += msg->payload_size;

            if (s_ota_total_size > 0) {
                uint8_t pct = static_cast<uint8_t>((static_cast<uint64_t>(s_ota_offset) * 100ULL) / s_ota_total_size);
                if (pct != s_ota_last_pct_log && (pct % 5 == 0 || s_ota_offset == s_ota_total_size)) {
                    s_ota_last_pct_log = pct;
                    ESP_LOGI(TAG, "[OTA] Progression : %u%% (%lu / %lu octets)",
                             pct,
                             static_cast<unsigned long>(s_ota_offset),
                             static_cast<unsigned long>(s_ota_total_size));
                }
            }

            if (msg->payload_size > 0 && msg->payload) {
                const void* clean_payload = nullptr;
                uint16_t    clean_size    = 0;
                ret = parse_ota_sub_element(s_ota_total_size, msg->payload, msg->payload_size, &clean_payload, &clean_size);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "[OTA] Erreur decodage sous-element : %s", esp_err_to_name(ret));
                    ota_abort_transfer();
                    ota_in_progress_.store(false);
                    return ret;
                }

                if (clean_size > 0) {
                    ret = esp_ota_write(s_ota_handle, clean_payload, clean_size);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "[OTA] Erreur esp_ota_write : %s", esp_err_to_name(ret));
                        ota_abort_transfer();
                        ota_in_progress_.store(false);
                        return ret;
                    }
                }
            }
            break;
        }

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "[OTA] Verification et application de l'image recue...");
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = (s_ota_offset == s_ota_total_size) ? ESP_OK : ESP_FAIL;
            ota_reset_transfer_state();
            ESP_LOGI(TAG, "[OTA] Verification integrite taille : %s", esp_err_to_name(ret));
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH: {
            ESP_LOGI(TAG, "[OTA] === Telechargement termine avec succes ! ===");
            ESP_LOGI(TAG, "[OTA] Nouvelle version : 0x%08lX | Manuf : 0x%04X | Type : 0x%04X | Taille : %lu octets | Duree : %ld s",
                     static_cast<unsigned long>(msg->ota_header.file_version),
                     msg->ota_header.manufacturer_code,
                     msg->ota_header.image_type,
                     static_cast<unsigned long>(msg->ota_header.image_size),
                     static_cast<long>((esp_timer_get_time() - s_ota_start_time_us) / 1000000LL));

            ota_in_progress_.store(false);
            if (!s_ota_handle || !s_ota_partition) {
                ESP_LOGE(TAG, "[OTA] Fin de transfert sans session OTA ouverte");
                return ESP_ERR_INVALID_STATE;
            }

            // esp_ota_end() vérifie l'image (magic, puce cible, somme de contrôle SHA-256) et libère la session
            ret = esp_ota_end(s_ota_handle);
            s_ota_handle = 0;
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "[OTA] Echec validation image esp_ota_end : %s", esp_err_to_name(ret));
                return ret;
            }

            ret = esp_ota_set_boot_partition(s_ota_partition);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "[OTA] Echec esp_ota_set_boot_partition : %s", esp_err_to_name(ret));
                return ret;
            }

            ESP_LOGW(TAG, "[OTA] Redemarrage de l'ESP32-C6 sur le nouveau firmware dans 1 seconde...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
            break;
        }

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_SERVER_NOT_FOUND:
            ESP_LOGW(TAG, "[OTA] Transfert OTA interrompu (statut=%d)", msg->upgrade_status);
            ota_abort_transfer();
            ota_in_progress_.store(false);
            break;

        default:
            ESP_LOGI(TAG, "[OTA] Statut OTA : %d", msg->upgrade_status);
            break;
    }

    return ret;
}

// ============================================================================
// Diagnostic & émission forcée de l'état complet
// ============================================================================

void ZbBridge::force_report_all() {
    if (!s_stack_running.load() || !connected_.load() || ota_in_progress_.load()) {
        return;
    }
    ZbLock lock;
    if (!lock) {
        zb_lock_timeouts_++;
        return;
    }
    ESP_LOGI(TAG, "[Zigbee] Emission forcee de l'etat complet (thermostats, consignes, etats) vers Home Assistant");

    // Cache de déduplication vidé : chaque attribut est ré-émis dès sa prochaine lecture BSB, et une
    // relecture complète de la table de scrutation est demandée (exécutée par la tâche propriétaire du bus).
    // Les modes et consignes ne sont PAS poussés tant qu'ils n'ont pas été relus depuis le démarrage : les
    // attributs ne contiennent alors que l'instantané de la découverte, parfois vieux de plusieurs semaines,
    // et Home Assistant verrait de fausses transitions (ECS « rallumée », consigne changée) à chaque redémarrage.
    report_cache_count_ = 0;
    resync_requested_.store(true);
    update_thermostat_running_state();

    // Les mesures indisponibles le sont redites ici : aucune lecture BSB ne viendra le faire à leur place
    for (size_t i = 0; i < watch_.unavailable_count(); ++i) {
        publish_unavailable(watch_.unavailable_field(i));
    }

    if (caps_.zone1.present && z1_mode_known_) {
        set_thermostat_system_mode(EP_MASTER_Z1, bsb_mode_to_zcl_system_mode(z1_mode_), false);
    }
    if (caps_.zone2.present && z2_mode_known_) {
        set_thermostat_system_mode(EP_ZONE2, bsb_mode_to_zcl_system_mode(z2_mode_), false);
    }
    if (caps_.has_dhw && dhw_mode_known_) {
        set_thermostat_system_mode(EP_DHW, dhw_mode_to_zcl_system_mode(dhw_mode_), false);
        set_on_off_attr(EP_DHW, dhw_mode_ != static_cast<uint8_t>(bsb::DhwMode::Off));
    }

    set_on_off_attr(EP_BACKUP_AUX, ext_antenna_enabled_);

    // État réel des sondes externes : après un redémarrage elles sont inactives (0), et Home Assistant doit
    // le voir pour renvoyer la température sans attendre (le blueprint réagit au passage à 0).
    if (caps_.zone1.present && has_ep_hydraulic1()) {
        set_analog_output_attr(EP_HYDRAULIC1, ext_sensor_z1_.active ? ext_sensor_z1_.temperature_c : 0.0f);
    }
    if (caps_.zone2.present && has_ep_hydraulic2()) {
        set_analog_output_attr(EP_HYDRAULIC2, ext_sensor_z2_.active ? ext_sensor_z2_.temperature_c : 0.0f);
    }
}

void ZbBridge::dump_diagnostics() {
    // Instantané pris sous verrou Zigbee, puis journalisé verrou relâché : écrire sur la console USB
    // peut être lent et ne doit pas retenir la pile Zigbee.
    struct Snapshot {
        bool     connected = false;
        uint16_t pan_id = 0, short_addr = 0;
        uint8_t  channel = 0;
        uint32_t sent = 0, suppressed = 0, failed = 0, lock_timeouts = 0, attr_errors = 0;
        bool     comp_running = false, backup_running = false, hp_pump = false, z1_pump = false, z2_pump = false;
        uint8_t  comp_mod = 0, backup_mod = 0, z1_mode = 0, z2_mode = 0, dhw_mode = 0;
        uint16_t hp_status = 0, z1_status = 0, z2_status = 0, dhw_status = 0;
        int8_t   z1_heating = -1, z2_heating = -1, dhw_heating = -1;
        int16_t  z1_demand = -1, z2_demand = -1, dhw_demand = -1;
        ExternalSensorState ext[2];
        bool     ext_antenna = false;
        bool     hp_temps_known = false;
        float    hp_flow_c = 0.0f, hp_return_c = 0.0f;
        bool     unfed_grace = false, unfed_alerted = false;
        size_t   watched = 0, unavailable = 0;
        uint32_t first_unavailable = 0;
    } s;

    {
        ZbLock lock(pdMS_TO_TICKS(200));
        if (!lock) {
            ESP_LOGW(TAG, "Diagnostic indisponible : pile Zigbee occupee.");
            return;
        }
        s.connected = connected_.load();
        if (stack_initialized_ && s.connected) {
            s.pan_id     = esp_zb_get_pan_id();
            s.channel    = esp_zb_get_current_channel();
            s.short_addr = esp_zb_get_short_address();
        }
        s.sent = zb_reports_sent_; s.suppressed = zb_reports_suppressed_; s.failed = zb_reports_failed_;
        s.lock_timeouts = zb_lock_timeouts_; s.attr_errors = zb_attr_errors_;
        s.comp_running = comp_running_; s.comp_mod = comp_mod_;
        s.backup_running = backup_running_; s.backup_mod = backup_mod_;
        s.hp_pump = hp_pump_running_; s.z1_pump = z1_pump_running_; s.z2_pump = z2_pump_running_;
        s.z1_mode = z1_mode_; s.z2_mode = z2_mode_; s.dhw_mode = dhw_mode_;
        s.hp_status = hp_status_code_; s.z1_status = z1_status_code_;
        s.z2_status = z2_status_code_; s.dhw_status = dhw_status_code_;
        s.z1_heating = z1_heating_.last_heating; s.z2_heating = z2_heating_.last_heating;
        s.dhw_heating = dhw_heating_.last_heating;
        s.z1_demand = z1_heating_.last_demand; s.z2_demand = z2_heating_.last_demand;
        s.dhw_demand = dhw_heating_.last_demand;
        s.ext[0] = ext_sensor_z1_; s.ext[1] = ext_sensor_z2_;
        s.ext_antenna = ext_antenna_enabled_;
        s.hp_temps_known = (hp_flow_ms_ != 0 && hp_return_ms_ != 0);
        s.hp_flow_c = hp_flow_c_; s.hp_return_c = hp_return_c_;
        s.unfed_grace = unfed_watch_.in_grace_period(); s.unfed_alerted = unfed_watch_.alerted();
        s.watched = watch_.tracked_count(); s.unavailable = watch_.unavailable_count();
        s.first_unavailable = watch_.unavailable_field(0);
    }

    // Une pompe que le régulateur ne connaît pas est dite « absente », pas « OFF »
    auto pump_text = [](bool present, bool running) { return !present ? "absente" : (running ? "ON" : "OFF"); };

    ESP_LOGI(TAG, "=== DIAGNOSTIC ZCL & BSB ===");
    ESP_LOGI(TAG, "Zigbee connecte: %s (PAN=0x%04X, Ch=%u, Addr=0x%04X)",
             s.connected ? "OUI" : "NON", s.pan_id, s.channel, s.short_addr);
    ESP_LOGI(TAG, "Rapports Zigbee: emis=%lu, suppr=%lu, err=%lu, lock_timeouts=%lu, attr_err=%lu",
             static_cast<unsigned long>(s.sent), static_cast<unsigned long>(s.suppressed),
             static_cast<unsigned long>(s.failed), static_cast<unsigned long>(s.lock_timeouts),
             static_cast<unsigned long>(s.attr_errors));
    if (s.unavailable == 0) {
        ESP_LOGI(TAG, "Mesures surveillees: %u, toutes fournies par le regulateur", static_cast<unsigned>(s.watched));
    } else {
        ESP_LOGI(TAG, "Mesures surveillees: %u, dont %u INDISPONIBLE(S) (%s%s)", static_cast<unsigned>(s.watched),
                 static_cast<unsigned>(s.unavailable), bsb::get_field_name(s.first_unavailable),
                 s.unavailable > 1 ? ", ..." : "");
    }
    ESP_LOGI(TAG, "Compresseur: %s, Modulation: %u %%, Appoint: %s %u %%, Statut PAC: %u (%s), Pompe PAC: %s",
             s.comp_running ? "ON" : "OFF", s.comp_mod,
             s.backup_running ? "ON" : "OFF", s.backup_mod,
             s.hp_status, bsb::get_hp_status_text_fr(s.hp_status),
             pump_text(caps_.has_hp_pump_state, s.hp_pump));
    if (s.hp_temps_known) {
        // L'écart départ - retour dit si la chaleur circule réellement (voir compute_heating_state())
        ESP_LOGI(TAG, "Depart PAC: %.1f C, Retour PAC: %.1f C, Ecart: %+.1f K%s",
                 s.hp_flow_c, s.hp_return_c, s.hp_flow_c - s.hp_return_c,
                 s.unfed_alerted ? " -> GENERATEUR DEMANDE SANS CIRCULATION : zones non alimentees"
                 : s.unfed_grace ? " -> depart pas encore plus chaud que le retour : chauffe des zones en attente"
                                 : "");
    }
    if (caps_.zone1.present) {
        ESP_LOGI(TAG, "Zone 1: mode BSB=%u (ZCL 0x%02X), Pompe=%s, Statut=%u (%s), Chauffage=%d, Demande=%d %%",
                 s.z1_mode, bsb_mode_to_zcl_system_mode(s.z1_mode),
                 pump_text(caps_.zone1.has_pump_state, s.z1_pump), s.z1_status, bsb::get_zone_status_text_fr(s.z1_status),
                 s.z1_heating, s.z1_demand);
    }
    if (caps_.zone2.present) {
        ESP_LOGI(TAG, "Zone 2: mode BSB=%u (ZCL 0x%02X), Pompe=%s, Statut=%u (%s), Chauffage=%d, Demande=%d %%",
                 s.z2_mode, bsb_mode_to_zcl_system_mode(s.z2_mode),
                 pump_text(caps_.zone2.has_pump_state, s.z2_pump), s.z2_status, bsb::get_zone_status_text_fr(s.z2_status),
                 s.z2_heating, s.z2_demand);
    }
    if (caps_.has_dhw) {
        ESP_LOGI(TAG, "Ballon ECS: mode BSB=%u (ZCL 0x%02X), Statut=%u (%s), Chauffage=%d, Demande=%d %%",
                 s.dhw_mode, dhw_mode_to_zcl_system_mode(s.dhw_mode),
                 s.dhw_status, bsb::get_dhw_status_text_fr(s.dhw_status),
                 s.dhw_heating, s.dhw_demand);
    }
    for (size_t i = 0; i < 2; ++i) {
        if (s.ext[i].active) {
            ESP_LOGI(TAG, "Sonde externe Zone %u: %.2f C, prise en compte par le regulateur: %s",
                     static_cast<unsigned>(i + 1), s.ext[i].temperature_c,
                     s.ext[i].confirmed ? "OUI" : "pas encore confirmee");
        }
    }
    ESP_LOGI(TAG, "Antenne RF: %s", s.ext_antenna ? "EXTERNE (U.FL)" : "INTERNE (Ceramique)");
    ESP_LOGI(TAG, "============================");
}

} // namespace zb_bsb

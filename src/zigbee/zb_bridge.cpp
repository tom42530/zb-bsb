#include "zb_bridge.h"
#include "board_config.h"
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
#include <cmath>
#include <cstring>

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

static uint64_t now_ms() {
    return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
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
    return static_cast<int16_t>(std::lround(temp_c * 100.0f));
}

static float zcl_s16_to_temp(int16_t val) {
    return static_cast<float>(val) / 100.0f;
}

ZbBridge& ZbBridge::instance() {
    static ZbBridge inst;
    return inst;
}

uint8_t ZbBridge::bsb_mode_to_zcl_system_mode(uint8_t bsb_mode) {
    switch (static_cast<bsb::HeatingMode>(bsb_mode)) {
        case bsb::HeatingMode::Protection:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   // 0x00 = Hors-gel / Off
        case bsb::HeatingMode::Auto:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO;  // 0x01 = Auto (programmation horaire PAC)
        case bsb::HeatingMode::Reduced:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP; // 0x09 = Réduit / Éco
        case bsb::HeatingMode::Comfort:
        default:
            return ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;  // 0x04 = Confort permanent
    }
}

uint8_t ZbBridge::zcl_system_mode_to_bsb_mode(uint8_t zcl_mode) {
    switch (zcl_mode) {
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF:
            return static_cast<uint8_t>(bsb::HeatingMode::Protection);
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO:
            return static_cast<uint8_t>(bsb::HeatingMode::Auto);
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP:
            return static_cast<uint8_t>(bsb::HeatingMode::Reduced);
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT:
        case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_EMERGENCY_HEATING:
            return static_cast<uint8_t>(bsb::HeatingMode::Comfort);
        default:
            return static_cast<uint8_t>(bsb::HeatingMode::Auto);
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
    uint8_t  running_mode    = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
    uint8_t  pi_demand       = 0;

    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, &unoccupied_heat);
    // On utilise les attributs AbsMin/Max en lecture seule pour définir les bornes 7-30 °C dans ZHA
    // sans créer d'entités number inutiles
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, &outdoor_s16);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, &remote_sensing);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &running_state);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID, &running_mode);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, &pi_demand);

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
    uint8_t zcl_sys_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;
    if (dhw_mode == static_cast<uint8_t>(bsb::DhwMode::Off)) {
        zcl_sys_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
    } else if (dhw_mode == static_cast<uint8_t>(bsb::DhwMode::Eco)) {
        zcl_sys_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP;
    }

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
    uint8_t  running_mode    = zcl_sys_mode;
    uint8_t  pi_demand       = 0;

    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, &unoccupied_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &abs_min_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &abs_max_heat);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &running_state);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID, &running_mode);
    esp_zb_thermostat_cluster_add_attr(attr, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, &pi_demand);

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
    esp_zb_bdb_set_scan_duration(3);
    // Puissance d'émission maximale (+20 dBm) pour un rôle Routeur Zigbee optimal
    esp_zb_set_tx_power(20);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

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

    if (stack_initialized_) {
        uint8_t on_val = enabled ? 1 : 0;
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
            esp_zb_zcl_set_attribute_val(EP_BACKUP_AUX,
                                         ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                         &on_val,
                                         false);
            esp_zb_lock_release();
        }
        report_attr(EP_BACKUP_AUX, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
    }
}

bool ZbBridge::begin(const bsb::BsbCapabilities& caps, bsb::BsbBus* bus, bool erase_zigbee_nvram) {
    caps_                = caps;
    bus_                 = bus;
    ext_antenna_enabled_ = load_external_antenna_nvs();
    board::set_xiao_rf_antenna(ext_antenna_enabled_);

    esp_zb_platform_config_t platform_config = {};
    platform_config.radio_config.radio_mode          = ZB_RADIO_MODE_NATIVE;
    platform_config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;

    ESP_RETURN_ON_ERROR(esp_zb_platform_config(&platform_config), TAG, "Echec esp_zb_platform_config");

    // Configuration en Routeur Zigbee 3.0 (alimenté sur secteur, répéteur mesh actif)
    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = 16;

    esp_zb_init(&zb_nwk_cfg);

    if (erase_zigbee_nvram) {
        ESP_LOGW(TAG, "Effacement NVRAM Zigbee demande (re-appairage)...");
        esp_zb_nvram_erase_at_start(true);
    }

    esp_zb_ep_list_t* ep_list = esp_zb_ep_list_create();
    build_endpoints(ep_list);

    ESP_RETURN_ON_ERROR(esp_zb_device_register(ep_list), TAG, "Echec esp_zb_device_register");
    esp_zb_core_action_handler_register(zb_action_handler);
    ESP_RETURN_ON_ERROR(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK),
                        TAG, "Echec esp_zb_set_primary_network_channel_set");

    stack_initialized_ = true;

    BaseType_t ok = xTaskCreate(zigbee_main_task, "zb_main", 8192, nullptr, 5, nullptr);
    return (ok == pdPASS);
}

void ZbBridge::handle_stack_signal(esp_zb_app_signal_t* signal_struct) {
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
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    connected_ = true;
                    ESP_LOGI(TAG, "Reconnecte au reseau Zigbee (PAN=0x%04X, Canal=%u, Addr=0x%04X)",
                             esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                    {
                        uint8_t on_val = ext_antenna_enabled_ ? 1 : 0;
                        esp_zb_zcl_set_attribute_val(EP_BACKUP_AUX,
                                                     ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                     ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                     &on_val,
                                                     false);
                    }
                    report_attr(EP_BACKUP_AUX, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
                    request_ota_update();
                }
            } else {
                ESP_LOGW(TAG, "Echec initialisation BDB (%s), nouvel essai dans 1s...", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                connected_ = true;
                ESP_LOGI(TAG, "Appairage Zigbee reussi ! (PAN=0x%04X, Canal=%u, Addr=0x%04X)",
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                {
                    uint8_t on_val = ext_antenna_enabled_ ? 1 : 0;
                    esp_zb_zcl_set_attribute_val(EP_BACKUP_AUX,
                                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                 &on_val,
                                                 false);
                }
                report_attr(EP_BACKUP_AUX, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
                request_ota_update();
            } else {
                connected_ = false;
                ESP_LOGI(TAG, "En attente d'ouverture du reseau Zigbee (ZHA / Z2M)... (%s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            connected_ = false;
            ESP_LOGW(TAG, "Appareil retire du reseau Zigbee, relance de l'appairage...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;

        default:
            break;
    }
}

// ============================================================================
// Gestion des sondes d'ambiance externes (Home Assistant ou Binding Zigbee)
// ============================================================================

void ZbBridge::set_external_room_temperature(uint8_t zone_index, float temp_c) {
    if (!bus_ || (zone_index != 1 && zone_index != 2)) return;

    const bool     is_z2       = (zone_index == 2);
    const uint8_t  thermo_ep   = is_z2 ? EP_ZONE2 : EP_MASTER_Z1;
    const uint8_t  ao_ep       = is_z2 ? EP_HYDRAULIC2 : EP_HYDRAULIC1;
    const uint32_t bsb_field   = is_z2 ? bsb::fields::Z2_ROOM_TEMP : bsb::fields::Z1_ROOM_TEMP;
    ExternalSensorState& state = is_z2 ? ext_sensor_z2_ : ext_sensor_z1_;

    // Une valeur < 4.0 °C (ex: 0.0 °C) désactive la sonde externe et rend la main au bus BSB
    if (temp_c < 4.0f) {
        if (state.active) {
            state.active = false;
            set_thermostat_u8_attr(thermo_ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, 0x00);
            ESP_LOGI(TAG, "Sonde d'ambiance externe Zone %u desactivee (retour sur mesure BSB).", zone_index);
        }
        return;
    }

    if (temp_c > 38.0f) {
        return;
    }

    const uint64_t t_now = now_ms();
    state.active          = true;
    state.temperature_c   = temp_c;
    state.last_update_ms  = t_now;
    state.last_bsb_inf_ms = t_now;

    ESP_LOGI(TAG, "[Sonde Externe Zone %u] Temp. Ambiante = %.2f C -> Thermostat EP%u & Bus BSB (INF 0x%08lX)",
             zone_index, temp_c, thermo_ep, static_cast<unsigned long>(bsb_field));

    // 1. Mettre à jour LocalTemperature et le bit RemoteSensing sur le Thermostat Zigbee (Zone 1 ou Zone 2)
    set_thermostat_s16_attr(thermo_ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, temp_c);
    set_thermostat_u8_attr(thermo_ep, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID,
                           ESP_ZB_ZCL_THERMOSTAT_REMOTE_SENSING_LOCAL_TEMPERATURE_SENSED_REMOTELY_BIT);
    if (is_z2) {
        set_temp_meas_attr(EP_ZONE2, temp_c);
    }

    // 2. Synchroniser l'attribut AnalogOutput correspondant
    set_analog_output_attr(ao_ep, temp_c);

    // 3. Diffuser immédiatement la température d'ambiance sur le bus BSB via une trame INF
    bus_->enqueue_inf_room_temp(bsb_field, temp_c);
}

void ZbBridge::loop_housekeeping() {
    if (!stack_initialized_ || !bus_) return;

    const uint64_t t_now = now_ms();

    // Maintenance Sonde Externe Zone 1
    if (ext_sensor_z1_.active) {
        if ((t_now - ext_sensor_z1_.last_update_ms) >= EXT_SENSOR_WATCHDOG_TIMEOUT_MS) {
            ext_sensor_z1_.active = false;
            set_thermostat_u8_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, 0x00);
            ESP_LOGW(TAG, "Watchdog Sonde Externe Zone 1 (aucune maj depuis 30 min) -> Retour sur mesure BSB.");
        } else if ((t_now - ext_sensor_z1_.last_bsb_inf_ms) >= EXT_SENSOR_BSB_REBROADCAST_MS) {
            ext_sensor_z1_.last_bsb_inf_ms = t_now;
            bus_->enqueue_inf_room_temp(bsb::fields::Z1_ROOM_TEMP, ext_sensor_z1_.temperature_c);
        }
    }

    // Maintenance Sonde Externe Zone 2
    if (ext_sensor_z2_.active) {
        if ((t_now - ext_sensor_z2_.last_update_ms) >= EXT_SENSOR_WATCHDOG_TIMEOUT_MS) {
            ext_sensor_z2_.active = false;
            set_thermostat_u8_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, 0x00);
            ESP_LOGW(TAG, "Watchdog Sonde Externe Zone 2 (aucune maj depuis 30 min) -> Retour sur mesure BSB.");
        } else if ((t_now - ext_sensor_z2_.last_bsb_inf_ms) >= EXT_SENSOR_BSB_REBROADCAST_MS) {
            ext_sensor_z2_.last_bsb_inf_ms = t_now;
            bus_->enqueue_inf_room_temp(bsb::fields::Z2_ROOM_TEMP, ext_sensor_z2_.temperature_c);
        }
    }
}

// ============================================================================
// Réception d'un rapport direct par Binding Zigbee (Capteur Temp -> EP1 / EP2)
// ============================================================================

esp_err_t ZbBridge::handle_attribute_report(const esp_zb_zcl_report_attr_message_t* msg) {
    if (!msg || msg->status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }

    if (msg->cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
        msg->attribute.id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
        msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16 &&
        msg->attribute.data.value != nullptr) {
        int16_t raw_s16 = *static_cast<const int16_t*>(msg->attribute.data.value);
        float temp_c    = zcl_s16_to_temp(raw_s16);

        if (msg->dst_endpoint == EP_MASTER_Z1 && caps_.zone1.present) {
            set_external_room_temperature(1, temp_c);
        } else if (msg->dst_endpoint == EP_ZONE2 && caps_.zone2.present) {
            set_external_room_temperature(2, temp_c);
        }
    }

    return ESP_OK;
}

// ============================================================================
// Réception des commandes d'écriture depuis Home Assistant (ZHA / Zigbee2MQTT)
// ============================================================================

esp_err_t ZbBridge::handle_attribute_write(const esp_zb_zcl_set_attr_value_message_t* msg) {
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS || !bus_) {
        return ESP_OK;
    }

    const uint8_t  ep         = msg->info.dst_endpoint;
    const uint16_t cluster_id = msg->info.cluster;
    const uint16_t attr_id    = msg->attribute.id;
    const void*    data_ptr   = msg->attribute.data.value;

    if (!data_ptr) return ESP_OK;

    // 1. Cluster Thermostat (0x0201) — Zones 1 & 2 + Ballon ECS
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        if (ep == EP_DHW) {
            if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
                int16_t raw_s16 = *static_cast<const int16_t*>(data_ptr);
                float temp_c    = zcl_s16_to_temp(raw_s16);
                if (temp_c >= 30.0f && temp_c <= 65.0f) {
                    ESP_LOGI(TAG, "[Zigbee->BSB] Ballon ECS Consigne Nominale (Thermostat) -> %.1f C", temp_c);
                    bus_->enqueue_set_temperature(bsb::fields::DHW_NOMINAL_SETPOINT, temp_c);
                    set_analog_output_attr(EP_DHW, temp_c);
                    report_attr(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
                }
            } else if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID) {
                int16_t raw_s16 = *static_cast<const int16_t*>(data_ptr);
                float temp_c    = zcl_s16_to_temp(raw_s16);
                if (temp_c >= 30.0f && temp_c <= 65.0f) {
                    ESP_LOGI(TAG, "[Zigbee->BSB] Ballon ECS Consigne Reduite (Thermostat) -> %.1f C", temp_c);
                    bus_->enqueue_set_temperature(bsb::fields::DHW_REDUCED_SETPOINT, temp_c);
                    report_attr(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
                }
            } else if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID) {
                uint8_t zcl_mode = *static_cast<const uint8_t*>(data_ptr);
                uint8_t dhw_mode = static_cast<uint8_t>(bsb::DhwMode::On);
                if (zcl_mode == ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF) {
                    dhw_mode = static_cast<uint8_t>(bsb::DhwMode::Off);
                } else if (zcl_mode == ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP) {
                    dhw_mode = static_cast<uint8_t>(bsb::DhwMode::Eco);
                }
                ESP_LOGI(TAG, "[Zigbee->BSB] Ballon ECS Mode (Thermostat) -> ZCL=0x%02X (BSB=%u)", zcl_mode, dhw_mode);
                bus_->enqueue_set_uint8(bsb::fields::DHW_MODE, dhw_mode);
                uint8_t on_val = (dhw_mode != static_cast<uint8_t>(bsb::DhwMode::Off)) ? 1 : 0;
                if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
                    esp_zb_zcl_set_attribute_val(EP_DHW,
                                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                 &on_val,
                                                 false);
                    esp_zb_lock_release();
                }
                report_attr(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
                report_attr(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
            }
            return ESP_OK;
        }

        const bool is_z2 = (ep == EP_ZONE2);
        const uint32_t comfort_field = is_z2 ? bsb::fields::Z2_COMFORT_SETPOINT : bsb::fields::Z1_COMFORT_SETPOINT;
        const uint32_t reduced_field = is_z2 ? bsb::fields::Z2_REDUCED_SETPOINT : bsb::fields::Z1_REDUCED_SETPOINT;
        const uint32_t mode_field    = is_z2 ? bsb::fields::Z2_MODE             : bsb::fields::Z1_MODE;
        const char*    zone_name     = is_z2 ? "Zone 2" : "Zone 1";

        if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            int16_t raw_s16 = *static_cast<const int16_t*>(data_ptr);
            float temp_c    = zcl_s16_to_temp(raw_s16);
            if (temp_c >= 7.0f && temp_c <= 30.0f) {
                ESP_LOGI(TAG, "[Zigbee->BSB] %s Consigne Confort -> %.1f C", zone_name, temp_c);
                bus_->enqueue_set_temperature(comfort_field, temp_c);
                report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
            }
        } else if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID) {
            int16_t raw_s16 = *static_cast<const int16_t*>(data_ptr);
            float temp_c    = zcl_s16_to_temp(raw_s16);
            if (temp_c >= 7.0f && temp_c <= 30.0f) {
                ESP_LOGI(TAG, "[Zigbee->BSB] %s Consigne Reduite -> %.1f C", zone_name, temp_c);
                bus_->enqueue_set_temperature(reduced_field, temp_c);
                set_analog_output_attr(ep, temp_c);
                report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
            }
        } else if (attr_id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID) {
            uint8_t zcl_mode = *static_cast<const uint8_t*>(data_ptr);
            uint8_t bsb_mode = zcl_system_mode_to_bsb_mode(zcl_mode);
            ESP_LOGI(TAG, "[Zigbee->BSB] %s Mode -> ZCL=0x%02X (BSB=%u)", zone_name, zcl_mode, bsb_mode);
            bus_->enqueue_set_uint8(mode_field, bsb_mode);
            report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
        }
    }

    // 2. Cluster AnalogOutput (0x000D) :
    //    - EP 1 : Consigne Réduite Zone 1 (712)
    //    - EP 2 : Consigne Réduite Zone 2 (1012)
    //    - EP 3 : Sonde Température Ambiante Externe Zone 1 (vers Thermostat EP1 + BSB INF 8740)
    //    - EP 4 : Sonde Température Ambiante Externe Zone 2 (vers Thermostat EP2 + BSB INF 8770)
    //    - EP 6 : Consigne Nominale Ballon ECS (1610)
    else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT &&
             attr_id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) {
        float val = *static_cast<const float*>(data_ptr);

        if (ep == EP_MASTER_Z1 && val >= 7.0f && val <= 30.0f) {
            ESP_LOGI(TAG, "[Zigbee->BSB] Zone 1 Consigne Reduite (Slider) -> %.1f C", val);
            bus_->enqueue_set_temperature(bsb::fields::Z1_REDUCED_SETPOINT, val);
            set_thermostat_s16_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, val);
        } else if (ep == EP_ZONE2 && val >= 7.0f && val <= 30.0f) {
            ESP_LOGI(TAG, "[Zigbee->BSB] Zone 2 Consigne Reduite (Slider) -> %.1f C", val);
            bus_->enqueue_set_temperature(bsb::fields::Z2_REDUCED_SETPOINT, val);
            set_thermostat_s16_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, val);
        } else if (ep == EP_HYDRAULIC1) {
            set_external_room_temperature(1, val);
        } else if (ep == EP_HYDRAULIC2) {
            set_external_room_temperature(2, val);
        } else if (ep == EP_DHW && val >= 30.0f && val <= 65.0f) {
            ESP_LOGI(TAG, "[Zigbee->BSB] Consigne Ballon ECS -> %.1f C", val);
            bus_->enqueue_set_temperature(bsb::fields::DHW_NOMINAL_SETPOINT, val);
            set_thermostat_s16_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, val);
        }
    }

    // 3. Cluster OnOff (0x0006) :
    //    - EP 6 : Activation Marche/Arrêt Ballon ECS (1600)
    //    - EP 7 : Sélection Antenne RF Zigbee Externe U.FL (OFF par défaut = Céramique interne, ON = U.FL externe)
    else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
             attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        bool on = *static_cast<const bool*>(data_ptr);
        if (ep == EP_DHW) {
            uint8_t dhw_mode = on ? static_cast<uint8_t>(bsb::DhwMode::On) : static_cast<uint8_t>(bsb::DhwMode::Off);
            ESP_LOGI(TAG, "[Zigbee->BSB] Mode Ballon ECS -> %s (%u)", on ? "Marche" : "Arret", dhw_mode);
            bus_->enqueue_set_uint8(bsb::fields::DHW_MODE, dhw_mode);
            set_thermostat_u8_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                   on ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF);
        } else if (ep == EP_BACKUP_AUX) {
            set_external_antenna_enabled(on);
        }
    }

    return ESP_OK;
}

// ============================================================================
// Mise à jour thread-safe des attributs ZCL (BSB -> Zigbee)
// ============================================================================

void ZbBridge::set_temp_meas_attr(uint8_t ep, float temp_c) {
    if (!stack_initialized_) return;
    int16_t val = temp_to_zcl_s16(temp_c);
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
}

void ZbBridge::set_binary_input_attr(uint8_t ep, bool state) {
    if (!stack_initialized_) return;
    uint8_t val = state ? 1 : 0;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID);
}

void ZbBridge::set_analog_input_attr(uint8_t ep, float val) {
    if (!stack_initialized_) return;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
}

void ZbBridge::set_analog_output_attr(uint8_t ep, float val) {
    if (!stack_initialized_) return;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID);
}

void ZbBridge::set_thermostat_s16_attr(uint8_t ep, uint16_t attr_id, float temp_c) {
    if (!stack_initialized_) return;
    int16_t val = temp_to_zcl_s16(temp_c);
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     attr_id,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
}

void ZbBridge::set_thermostat_u8_attr(uint8_t ep, uint16_t attr_id, uint8_t val) {
    if (!stack_initialized_) return;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     attr_id,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
}

void ZbBridge::set_thermostat_u16_attr(uint8_t ep, uint16_t attr_id, uint16_t val) {
    if (!stack_initialized_) return;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        esp_zb_zcl_set_attribute_val(ep,
                                     ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     attr_id,
                                     &val,
                                     false);
        esp_zb_lock_release();
    }
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, attr_id);
}

void ZbBridge::report_attr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id) {
    if (!connected_ || ota_in_progress_) return;

    esp_zb_zcl_attr_location_info_t info = {
        .endpoint_id  = ep,
        .cluster_id   = cluster_id,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .manuf_code   = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .attr_id      = attr_id,
    };

    if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        // S'assurer que l'entrée reporting_info existe dans ZBOSS avec profile_id = 0x0104 (HA)
        // avant d'appeler esp_zb_zcl_report_attr_cmd_req (qui vérifie sa présence).
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

        // Envoi unicast direct au Coordinateur (0x0000, EP 1) :
        // Contournement total de la limite matérielle ZBOSS de 16 entrées dans la table APS Binding,
        // garantissant que les Endpoints 5, 6 (ECS) et 7 remontent leurs rapports en temps réel.
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

        esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&cmd);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "report_attr cmd failed EP%u Cl 0x%04X Attr 0x%04X: %s",
                     ep, cluster_id, attr_id, esp_err_to_name(ret));
        }
        esp_zb_lock_release();
    }
}

void ZbBridge::update_thermostat_running_state() {
    if (caps_.zone1.present) {
        const bool z1_circulating = caps_.has_hp_pump_state ? hp_pump_running_ : z1_pump_running_;
        const bool z1_heating = z1_circulating && (comp_running_ || !caps_.has_compressor_state);
        set_thermostat_u16_attr(EP_MASTER_Z1,
                                ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
                                z1_heating ? 0x0001 : 0x0000);
        set_thermostat_u8_attr(EP_MASTER_Z1,
                               ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
                               z1_heating ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF);
    }
    if (caps_.zone2.present) {
        const bool z2_heating = z2_pump_running_ && (comp_running_ || !caps_.has_compressor_state);
        set_thermostat_u16_attr(EP_ZONE2,
                                ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
                                z2_heating ? 0x0001 : 0x0000);
        set_thermostat_u8_attr(EP_ZONE2,
                               ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
                               z2_heating ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF);
    }
}

void ZbBridge::on_bsb_value(const bsb::DecodedValue& val) {
    if (!val.valid || !stack_initialized_) return;

    using namespace bsb::fields;

    switch (val.field_id) {
        // --- Capteurs généraux & Hydrauliques PAC ---
        case OUTDOOR_TEMP:
            set_temp_meas_attr(EP_MASTER_Z1, val.float_val);
            if (caps_.zone1.present) {
                set_thermostat_s16_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, val.float_val);
            }
            if (caps_.zone2.present) {
                set_thermostat_s16_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, val.float_val);
            }
            break;

        case HP_FLOW_TEMP:
            if (caps_.has_hp_flow_temp) {
                set_temp_meas_attr(EP_HYDRAULIC1, val.float_val);
            }
            break;

        case HP_RETURN_TEMP:
            if (caps_.has_hp_return_temp) {
                set_temp_meas_attr(EP_HYDRAULIC2, val.float_val);
            }
            break;

        case COMPRESSOR_STATE:
            comp_running_ = val.bool_val;
            set_binary_input_attr(EP_MASTER_Z1, val.bool_val);
            update_thermostat_running_state();
            break;

        case HP_PUMP_STATE:
            hp_pump_running_ = val.bool_val;
            z1_pump_running_ = val.bool_val;
            if (caps_.has_hp_pump_state || caps_.zone1.has_pump_state) {
                set_binary_input_attr(EP_HYDRAULIC1, val.bool_val);
            }
            update_thermostat_running_state();
            break;

        case COMPRESSOR_MOD: {
            uint8_t demand = static_cast<uint8_t>(val.int_val > 100 ? 100 : (val.int_val < 0 ? 0 : val.int_val));
            set_analog_input_attr(EP_MASTER_Z1, static_cast<float>(demand));
            if (caps_.zone1.present) {
                set_thermostat_u8_attr(EP_MASTER_Z1, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, demand);
            }
            if (caps_.zone2.present) {
                set_thermostat_u8_attr(EP_ZONE2, ESP_ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, demand);
            }
            break;
        }

        case BACKUP_HEATER_STATE:
            if (caps_.has_backup_heater) {
                set_binary_input_attr(EP_BACKUP_AUX, val.bool_val);
            }
            break;

        case BACKUP_MOD:
            if (caps_.has_backup_mod) {
                set_analog_input_attr(EP_BACKUP_AUX, val.float_val);
            }
            break;

        case HP_STATUS_CODE:
            if (caps_.has_hp_status_code) {
                ESP_LOGD(TAG, "Statut PAC (8006) = %ld (%s)",
                         static_cast<long>(val.int_val),
                         bsb::get_hp_status_text_fr(static_cast<uint16_t>(val.int_val)));
                set_analog_input_attr(EP_HYDRAULIC1, val.float_val);
            }
            break;

        // --- Zone 1 ---
        case Z1_MODE:
            if (caps_.zone1.present) {
                set_thermostat_u8_attr(EP_MASTER_Z1,
                                       ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                       bsb_mode_to_zcl_system_mode(static_cast<uint8_t>(val.int_val)));
            }
            break;

        case Z1_COMFORT_SETPOINT:
            if (caps_.zone1.present) {
                set_thermostat_s16_attr(EP_MASTER_Z1,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                        val.float_val);
            }
            break;

        case Z1_REDUCED_SETPOINT:
            if (caps_.zone1.present) {
                set_thermostat_s16_attr(EP_MASTER_Z1,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID,
                                        val.float_val);
                set_analog_output_attr(EP_MASTER_Z1, val.float_val);
            }
            break;

        case Z1_ACTIVE_SETPOINT:
            if (caps_.zone1.present && !caps_.zone1.has_room_sensor && !ext_sensor_z1_.active) {
                // Sans sonde d'ambiance filaire ni sonde externe active, afficher la consigne active
                set_thermostat_s16_attr(EP_MASTER_Z1,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                        val.float_val);
            }
            break;

        case Z1_ROOM_TEMP:
            if (caps_.zone1.has_room_sensor && val.float_val < 45.0f) {
                set_temp_meas_attr(EP_BACKUP_AUX, val.float_val);
                if (!ext_sensor_z1_.active) {
                    set_thermostat_s16_attr(EP_MASTER_Z1,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                            val.float_val);
                }
            }
            break;

        case Z1_FLOW_TEMP:
            if (caps_.zone1.has_flow_sensor && !caps_.zone2.has_flow_sensor) {
                set_temp_meas_attr(EP_FLOW_Z2, val.float_val);
            }
            break;

        case Z1_PUMP_STATE:
            if (!caps_.has_hp_pump_state) {
                z1_pump_running_ = val.bool_val;
                if (caps_.zone1.has_pump_state) {
                    set_binary_input_attr(EP_HYDRAULIC1, val.bool_val);
                }
                update_thermostat_running_state();
            }
            break;

        case Z1_STATUS_CODE:
            if (caps_.zone1.has_status_code) {
                set_analog_input_attr(EP_HYDRAULIC2, val.float_val);
            }
            break;

        // --- Zone 2 ---
        case Z2_MODE:
            if (caps_.zone2.present) {
                set_thermostat_u8_attr(EP_ZONE2,
                                       ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                       bsb_mode_to_zcl_system_mode(static_cast<uint8_t>(val.int_val)));
            }
            break;

        case Z2_COMFORT_SETPOINT:
            if (caps_.zone2.present) {
                set_thermostat_s16_attr(EP_ZONE2,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                        val.float_val);
            }
            break;

        case Z2_REDUCED_SETPOINT:
            if (caps_.zone2.present) {
                set_thermostat_s16_attr(EP_ZONE2,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID,
                                        val.float_val);
                set_analog_output_attr(EP_ZONE2, val.float_val);
            }
            break;

        case Z2_ACTIVE_SETPOINT:
            if (caps_.zone2.present && (caps_.zone2.has_flow_sensor || caps_.zone1.has_flow_sensor)) {
                set_analog_input_attr(EP_FLOW_Z2, val.float_val);
            }
            break;

        case Z2_ROOM_TEMP:
            if (caps_.zone2.present && caps_.zone2.has_room_sensor && val.float_val < 45.0f) {
                set_temp_meas_attr(EP_ZONE2, val.float_val);
                if (!ext_sensor_z2_.active) {
                    set_thermostat_s16_attr(EP_ZONE2,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                            val.float_val);
                }
            }
            break;

        case Z2_FLOW_TEMP:
            if (caps_.zone2.has_flow_sensor) {
                set_temp_meas_attr(EP_FLOW_Z2, val.float_val);
            }
            break;

        case Z2_PUMP_STATE:
            z2_pump_running_ = val.bool_val;
            if (caps_.zone2.has_pump_state) {
                set_binary_input_attr(EP_ZONE2, val.bool_val);
            }
            update_thermostat_running_state();
            break;

        case Z2_STATUS_CODE:
            if (caps_.zone2.has_status_code) {
                set_analog_input_attr(EP_ZONE2, val.float_val);
            }
            break;

        // --- Ballon ECS (DHW) ---
        case DHW_TANK_TEMP:
            if (caps_.has_dhw && caps_.has_dhw_tank_temp) {
                set_temp_meas_attr(EP_DHW, val.float_val);
                set_thermostat_s16_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, val.float_val);
            }
            break;

        case DHW_NOMINAL_SETPOINT:
            if (caps_.has_dhw) {
                set_analog_output_attr(EP_DHW, val.float_val);
                set_thermostat_s16_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, val.float_val);
            }
            break;

        case DHW_REDUCED_SETPOINT:
            if (caps_.has_dhw) {
                set_thermostat_s16_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, val.float_val);
            }
            break;

        case DHW_MODE:
            if (caps_.has_dhw) {
                uint8_t zcl_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;
                if (val.int_val == static_cast<int32_t>(bsb::DhwMode::Off)) {
                    zcl_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
                } else if (val.int_val == static_cast<int32_t>(bsb::DhwMode::Eco)) {
                    zcl_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_SLEEP;
                }
                set_thermostat_u8_attr(EP_DHW, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, zcl_mode);

                uint8_t on_val = (val.int_val != static_cast<int32_t>(bsb::DhwMode::Off)) ? 1 : 0;
                if (esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
                    esp_zb_zcl_set_attribute_val(EP_DHW,
                                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                                 &on_val,
                                                 false);
                    esp_zb_lock_release();
                }
                report_attr(EP_DHW, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
            }
            break;

        case DHW_STATUS_CODE:
            if (caps_.has_dhw) {
                set_analog_input_attr(EP_DHW, val.float_val);
                // Codes de chauffe active ECS :
                // 61 = Charge forcée légionellose, 63 = Charge ECS active, 66 = Charge appoint, 97 = En charge
                int32_t code = static_cast<int32_t>(val.float_val);
                bool dhw_heating = (code == 61 || code == 63 || code == 66 || code == 97);
                set_thermostat_u16_attr(EP_DHW,
                                        ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
                                        dhw_heating ? 0x0001 : 0x0000);
                set_thermostat_u8_attr(EP_DHW,
                                       ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
                                       dhw_heating ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF);
            }
            break;

        default:
            break;
    }
}

// ============================================================================
// Support Mise à Jour Firmware OTA Zigbee (ZCL 0x0019 + zha_toolkit.ota_notify)
// ============================================================================

static constexpr uint16_t OTA_ELEMENT_HEADER_LEN = 6; // [uint16_t tag_id, uint32_t length]
static constexpr uint16_t OTA_UPGRADE_IMAGE_TAG  = 0x0000;

static const esp_partition_t* s_ota_partition     = nullptr;
static esp_ota_handle_t       s_ota_handle        = 0;
static bool                   s_ota_tag_received  = false;
static uint32_t               s_ota_total_size    = 0;
static uint32_t               s_ota_offset        = 0;
static int64_t                s_ota_start_time_us = 0;
static uint8_t                s_ota_last_pct_log  = 255;

static esp_err_t parse_ota_sub_element(uint32_t total_size,
                                       const void* payload,
                                       uint16_t payload_size,
                                       const void** out_buf,
                                       uint16_t* out_len) {
    static uint16_t s_tag_id = 0;

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

        if ((length + OTA_ELEMENT_HEADER_LEN) != total_size) {
            ESP_LOGE(TAG, "[OTA] Longueur sous-element invalide (%lu + 6 != %lu)",
                     static_cast<unsigned long>(length),
                     static_cast<unsigned long>(total_size));
            return ESP_ERR_INVALID_ARG;
        }

        s_tag_id = tag_id;
        s_ota_tag_received = true;

        *out_buf = ptr + OTA_ELEMENT_HEADER_LEN;
        *out_len = payload_size - OTA_ELEMENT_HEADER_LEN;
    } else {
        *out_buf = payload;
        *out_len = payload_size;
    }

    if (s_tag_id != OTA_UPGRADE_IMAGE_TAG) {
        ESP_LOGE(TAG, "[OTA] Tag de sous-element non supporte : 0x%04X", s_tag_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void find_ota_server_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void* user_ctx) {
    (void)user_ctx;
    uint16_t srv_addr = (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) ? addr : 0x0000;
    uint8_t  srv_ep   = (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) ? endpoint : 1;

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
    } else {
        ESP_LOGD(TAG, "[OTA] Aucune mise a jour OTA en attente (statut ZCL=0x%02X)", msg->info.status);
    }
    return ESP_OK;
}

esp_err_t ZbBridge::handle_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* msg) {
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;

    switch (msg->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START: {
            ESP_LOGI(TAG, "[OTA] === Demarrage de la mise a jour firmware Zigbee OTA ===");
            ota_in_progress_    = true;
            s_ota_offset        = 0;
            s_ota_total_size    = 0;
            s_ota_tag_received  = false;
            s_ota_last_pct_log  = 255;
            s_ota_start_time_us = esp_timer_get_time();

            s_ota_partition = esp_ota_get_next_update_partition(nullptr);
            if (!s_ota_partition) {
                ESP_LOGE(TAG, "[OTA] Aucune partition OTA disponible !");
                ota_in_progress_ = false;
                return ESP_FAIL;
            }

            ESP_LOGI(TAG, "[OTA] Ecriture vers la partition '%s' (offset=0x%08lX, taille=%lu)",
                     s_ota_partition->label,
                     static_cast<unsigned long>(s_ota_partition->address),
                     static_cast<unsigned long>(s_ota_partition->size));

            ret = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "[OTA] Echec esp_ota_begin : %s", esp_err_to_name(ret));
                ota_in_progress_ = false;
                return ret;
            }
            break;
        }

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE: {
            s_ota_total_size = msg->ota_header.image_size;
            s_ota_offset    += msg->payload_size;

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
                    if (s_ota_handle) {
                        esp_ota_abort(s_ota_handle);
                        s_ota_handle = 0;
                    }
                    ota_in_progress_ = false;
                    return ret;
                }

                if (clean_size > 0) {
                    ret = esp_ota_write(s_ota_handle, clean_payload, clean_size);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "[OTA] Erreur esp_ota_write : %s", esp_err_to_name(ret));
                        esp_ota_abort(s_ota_handle);
                        s_ota_handle     = 0;
                        ota_in_progress_ = false;
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
            s_ota_offset       = 0;
            s_ota_total_size   = 0;
            s_ota_tag_received = false;
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

            ota_in_progress_ = false;
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
            if (s_ota_handle) {
                esp_ota_abort(s_ota_handle);
                s_ota_handle = 0;
            }
            s_ota_offset       = 0;
            s_ota_total_size   = 0;
            s_ota_tag_received = false;
            ota_in_progress_   = false;
            break;

        default:
            ESP_LOGI(TAG, "[OTA] Statut OTA : %d", msg->upgrade_status);
            break;
    }

    return ret;
}

} // namespace zb_bsb

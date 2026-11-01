#pragma once

#include <cstdint>
#include "bsb_types.h"
#include "bsb_bus.h"
#include "esp_zigbee_core.h"

namespace zb_bsb {

/**
 * @brief Pont dynamique BSB <-> Zigbee 3.0 (Compatible ZHA & Zigbee2MQTT).
 *
 * Construit dynamiquement l'arbre d'Endpoints et de Clusters ZCL en fonction
 * des capacités matérielles découvertes sur la pompe à chaleur (`BsbCapabilities`).
 *
 * Supporte l'injection d'une sonde de température d'ambiance externe depuis
 * Home Assistant pour chaque Thermostat (Zone 1 et Zone 2) :
 *   1. Via une entité `number` dédiée (`AnalogOutput` 0x000D sur EP3 pour Zone 1, EP4 pour Zone 2)
 *   2. Via Binding Zigbee direct d'un capteur `TemperatureMeasurement` (0x0402) sur EP1 ou EP2
 *   3. Avec ré-émission périodique automatique sur le bus BSB (trame INF toutes les 60s)
 *      et watchdog de sécurité (retour sur sonde BSB si aucune mise à jour pendant 30 min).
 */
class ZbBridge {
public:
    // Numérotation fixe et déterministe des Endpoints dynamiques
    static constexpr uint8_t EP_MASTER_Z1  = 1; // Maître PAC + Extérieur + Compresseur + Thermostat Zone 1
    static constexpr uint8_t EP_ZONE2      = 2; // Thermostat Zone 2 + Ambiance Zone 2 + Pompe Q6 + Statut Z2
    static constexpr uint8_t EP_HYDRAULIC1 = 3; // Départ PAC + Pompe Q2 Z1 + Statut PAC + Sonde Externe Zone 1
    static constexpr uint8_t EP_HYDRAULIC2 = 4; // Retour PAC + Statut Zone 1 + Sonde Externe Zone 2
    static constexpr uint8_t EP_FLOW_Z2    = 5; // Départ Zone 2 + Consigne Active Zone 2
    static constexpr uint8_t EP_DHW        = 6; // Ballon ECS : Température + Consigne + Switch Marche/Arrêt + Statut
    static constexpr uint8_t EP_BACKUP_AUX = 7; // Appoint Électrique + Modulation + Sonde Ambiance BSB Z1 (si présente)

    static ZbBridge& instance();

    /**
     * @brief Initialise la pile Zigbee Router, construit les endpoints découverts
     *        et démarre la tâche FreeRTOS Zigbee.
     */
    bool begin(const bsb::BsbCapabilities& caps, bsb::BsbBus* bus, bool erase_zigbee_nvram = false);

    /**
     * @brief Met à jour les attributs ZCL et notifie le coordinateur lorsqu'une
     *        nouvelle valeur est reçue depuis le bus BSB.
     */
    void on_bsb_value(const bsb::DecodedValue& val);

    /**
     * @brief Maintenance périodique (à appeler depuis loop()) :
     *        - Ré-émet toutes les 60s sur le bus BSB (INF) les températures d'ambiance
     *          externes reçues de Home Assistant (nécessaire pour que le régulateur
     *          Siemens RVS21 conserve la mesure externe active).
     *        - Désactive la sonde externe si Home Assistant ne l'a pas rafraîchie depuis > 30 min.
     */
    void loop_housekeeping();

    /**
     * @brief Définit ou met à jour la température d'ambiance externe d'une zone (1 ou 2).
     */
    void set_external_room_temperature(uint8_t zone_index, float temp_c);

    /**
     * @brief Charge depuis la NVS le choix d'antenne RF (false = interne par défaut, true = U.FL externe).
     */
    static bool load_external_antenna_nvs();

    /**
     * @brief Active ou désactive à chaud l'antenne Zigbee externe U.FL (GPIO14) et sauvegarde en NVS.
     */
    void set_external_antenna_enabled(bool enabled);

    /**
     * @brief Indique si l'antenne externe U.FL est actuellement sélectionnée.
     */
    bool is_external_antenna_enabled() const { return ext_antenna_enabled_; }

    /**
     * @brief Indique si le nœud est actuellement connecté au réseau Zigbee.
     */
    bool is_connected() const { return connected_; }

    /**
     * @brief Indique si un téléchargement de mise à jour firmware OTA Zigbee est en cours.
     */
    bool is_ota_in_progress() const { return ota_in_progress_; }

    /**
     * @brief Interroge le serveur OTA du coordinateur Zigbee (Cluster 0x0019).
     */
    void request_ota_update();

    /**
     * @brief Conversion bidirectionnelle des modes de chauffage Siemens BSB <-> Zigbee ZCL SystemMode.
     */
    static uint8_t bsb_mode_to_zcl_system_mode(uint8_t bsb_mode);
    static uint8_t zcl_system_mode_to_bsb_mode(uint8_t zcl_mode);


    /**
     * @brief Gestionnaire interne des signaux de la pile ZBOSS.
     */
    void handle_stack_signal(esp_zb_app_signal_t* signal_struct);

    /**
     * @brief Gestionnaire interne des écritures d'attributs ZCL (Home Assistant -> PAC).
     */
    esp_err_t handle_attribute_write(const esp_zb_zcl_set_attr_value_message_t* message);

    /**
     * @brief Gestionnaire interne des rapports d'attributs ZCL (Binding direct capteur Zigbee -> Thermostat).
     */
    esp_err_t handle_attribute_report(const esp_zb_zcl_report_attr_message_t* message);

    /**
     * @brief Gestionnaire interne de réception des blocs de mise à jour firmware OTA Zigbee (0x0019).
     */
    esp_err_t handle_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* message);

    /**
     * @brief Gestionnaire interne de réponse Query Next Image OTA Zigbee (0x0019).
     */
    esp_err_t handle_ota_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* message);

private:
    ZbBridge() = default;

    struct ExternalSensorState {
        bool     active          = false;
        float    temperature_c   = 0.0f;
        uint64_t last_update_ms  = 0;
        uint64_t last_bsb_inf_ms = 0;
    };

    void build_endpoints(esp_zb_ep_list_t* ep_list);
    void update_thermostat_running_state();

    void set_temp_meas_attr(uint8_t ep, float temp_c);
    void set_binary_input_attr(uint8_t ep, bool state);
    void set_analog_input_attr(uint8_t ep, float val);
    void set_analog_output_attr(uint8_t ep, float val);
    void set_thermostat_s16_attr(uint8_t ep, uint16_t attr_id, float temp_c);
    void set_thermostat_u8_attr(uint8_t ep, uint16_t attr_id, uint8_t val);
    void set_thermostat_u16_attr(uint8_t ep, uint16_t attr_id, uint16_t val);
    void report_attr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id);



    bsb::BsbCapabilities caps_              = {};
    bsb::BsbBus*         bus_               = nullptr;
    bool                 stack_initialized_ = false;
    bool                 connected_         = false;
    volatile bool        ota_in_progress_   = false;

    bool                 comp_running_        = false;
    bool                 hp_pump_running_     = false;
    bool                 z1_pump_running_     = false;
    bool                 z2_pump_running_     = false;
    bool                 ext_antenna_enabled_ = false;

    ExternalSensorState  ext_sensor_z1_       = {};
    ExternalSensorState  ext_sensor_z2_       = {};
};

} // namespace zb_bsb

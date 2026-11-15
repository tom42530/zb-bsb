#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include "bsb_types.h"
#include "bsb_bus.h"
#include "bsb_freshness.h"
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
 *
 * Modèle de concurrence :
 *   L'état interne du pont est protégé par le verrou de la pile Zigbee. Les callbacks de la pile
 *   s'exécutent dans la tâche Zigbee (verrou déjà tenu) ; les points d'entrée appelés depuis la
 *   tâche Arduino (`on_bsb_value`, `loop_housekeeping`, `force_report_all`, `dump_diagnostics`...)
 *   prennent ce verrou une seule fois pour toute la durée du traitement.
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
     *        nouvelle valeur est reçue depuis le bus BSB (appelé par la tâche propriétaire du bus).
     */
    void on_bsb_value(const bsb::DecodedValue& val);

    /**
     * @brief Maintenance périodique (à appeler depuis loop()) :
     *        - Ré-émet toutes les 60s sur le bus BSB (INF) les températures d'ambiance
     *          externes reçues de Home Assistant (nécessaire pour que le régulateur
     *          Siemens RVS21 conserve la mesure externe active).
     *        - Désactive la sonde externe si Home Assistant ne l'a pas rafraîchie depuis > 30 min.
     *        - Resynchronise l'état de chauffe des thermostats et surveille un transfert OTA bloqué.
     */
    void loop_housekeeping();

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
    bool is_connected() const { return connected_.load(); }

    /**
     * @brief Indique si l'appareil n'est rattaché à aucun réseau (état usine) et attend un appairage.
     */
    bool is_awaiting_pairing() const { return awaiting_pairing_.load(); }

    /**
     * @brief Indique si la pile Zigbee a démarré (une éventuelle purge de sa NVRAM est alors faite).
     */
    bool is_stack_started() const;

    /**
     * @brief Indique si un échange radio aller-retour avec le coordinateur a été constaté depuis le
     *        démarrage (réponse à une de nos requêtes, ou écriture reçue). « Connecté » ne le prouve
     *        pas : un routeur déjà appairé retrouve son réseau en relisant simplement sa mémoire.
     */
    bool coordinator_reached() const { return coordinator_reached_.load(); }
    void note_coordinator_reached() { coordinator_reached_.store(true); }

    /**
     * @brief Indique si le coordinateur a demandé l'identification visuelle de l'appareil (cluster Identify).
     */
    bool is_identifying() const;

    /**
     * @brief Métriques de santé et de débit du reporting Zigbee 3.0.
     */
    uint32_t reports_sent() const { return zb_reports_sent_; }
    uint32_t reports_suppressed() const { return zb_reports_suppressed_; }
    uint32_t reports_failed() const { return zb_reports_failed_; }
    uint32_t lock_timeouts() const { return zb_lock_timeouts_; }

    /**
     * @brief Indique si un téléchargement de mise à jour firmware OTA Zigbee est en cours.
     */
    bool is_ota_in_progress() const { return ota_in_progress_.load(); }

    /**
     * @brief Réinitialisation usine Zigbee : efface la partition `zb_storage` puis redémarre.
     */
    void factory_reset();

    /**
     * @brief Conversion bidirectionnelle des modes de chauffage Siemens BSB <-> Zigbee ZCL SystemMode.
     */
    static uint8_t bsb_mode_to_zcl_system_mode(uint8_t bsb_mode);
    /** @return false si le mode ZCL n'a pas d'équivalent sur un circuit de chauffage (bsb_mode est alors inchangé). */
    static bool    zcl_system_mode_to_bsb_mode(uint8_t zcl_mode, uint8_t& bsb_mode);

    /**
     * @brief Empreinte de la disposition Zigbee (endpoints / clusters) induite par une topologie PAC.
     *        Si elle change, les tables de reporting et de binding mémorisées par la pile ne
     *        correspondent plus aux clusters exposés : la NVRAM Zigbee doit être purgée.
     */
    static uint32_t layout_signature(const bsb::BsbCapabilities& caps);

    void dump_diagnostics();
    void force_report_all();

    // --- Points d'entrée des callbacks C de la pile Zigbee (tâche Zigbee) ---
    void      handle_stack_signal(esp_zb_app_signal_t* signal_struct);
    esp_err_t handle_attribute_write(const esp_zb_zcl_set_attr_value_message_t* message);
    esp_err_t handle_attribute_report(const esp_zb_zcl_report_attr_message_t* message);
    esp_err_t handle_ota_upgrade_status(const esp_zb_zcl_ota_upgrade_value_message_t* message);
    esp_err_t handle_ota_query_image_resp(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t* message);

private:
    ZbBridge() = default;

    struct ExternalSensorState {
        bool     active          = false;
        bool     confirmed       = false; // Le régulateur relit bien la valeur injectée (Prog 8740 / 8770)
        bool     warned          = false; // Avertissement « non prise en compte » déjà journalisé
        bool     from_binding    = false; // Alimentée par un capteur lié en direct (watchdog plus long)
        float    temperature_c   = 0.0f;
        uint64_t activated_ms    = 0;
        uint64_t last_match_ms   = 0;     // Dernière relecture concordante de 8740 / 8770
        uint64_t last_update_ms  = 0;
        uint64_t last_bsb_inf_ms = 0;
    };

    // Dernière valeur connue du coordinateur pour un attribut (rapport émis ou écriture reçue)
    struct ReportCacheEntry {
        uint8_t  ep             = 0;
        uint16_t cluster_id     = 0;
        uint16_t attr_id        = 0;
        float    value          = 0.0f;
        uint64_t last_report_ms = 0;
    };
    static constexpr size_t   MAX_REPORT_CACHE     = 64;
    static constexpr uint16_t BINDING_TABLE_SIZE   = 32;     // Liaisons APS source et destination
    static constexpr uint32_t DEFAULT_HEARTBEAT_MS = 300000; // Ré-émission d'une valeur inchangée : 5 min
    static constexpr uint32_t STATE_HEARTBEAT_MS   = 60000;  // État de chauffe / mode des thermostats : 1 min

    void build_endpoints(esp_zb_ep_list_t* ep_list);
    void on_network_joined();
    void request_ota_update();

    // --- État de chauffe des thermostats ---
    /** Publication de l'état de chauffe d'un thermostat : filtre de confirmation et dernier état émis. */
    struct HeatingPublication {
        bsb::HeatingConfirmation filter;
        int8_t  last_heating = -1; // -1 = jamais publié
        int16_t last_demand  = -1;
    };
    void update_thermostat_running_state();
    void sync_thermostat_action(uint8_t ep, const char* label, const bsb::ThermostatAction& computed,
                                uint64_t t_now, HeatingPublication& pub);
    void note_zone_evidence(const char* label, uint16_t status, bsb::ZoneEvidence evidence, uint16_t& logged_code);
    void forget_heat_pump_state();

    // --- Mesures que le régulateur ne fournit plus ---
    // Une mesure qui n'arrive plus (bus coupé, régulateur hors tension, sonde en défaut) n'est pas laissée
    // figée sur sa dernière valeur : elle est publiée « indisponible » (valeur invalide ZCL), et revient
    // d'elle-même à la lecture suivante.
    void track_measurements(uint64_t t_now);
    void publish_unavailable(uint32_t field_id);
    void expire_stale_measurements(uint64_t t_now);

    // --- Sondes d'ambiance externes ---
    void set_external_room_temperature(uint8_t zone_index, float temp_c, bool from_binding);
    void deactivate_external_sensor(uint8_t zone_index, const char* reason);
    void maintain_external_sensor(uint8_t zone_index, uint64_t t_now);
    void check_external_sensor_feedback(uint8_t zone_index, float controller_room_c);

    // --- Écritures reçues du coordinateur ---
    void handle_zone_thermostat_write(uint8_t ep, uint16_t attr_id, const void* data);
    void handle_dhw_thermostat_write(uint16_t attr_id, const void* data);
    void handle_analog_output_write(uint8_t ep, const void* data);
    void handle_on_off_write(uint8_t ep, const void* data);
    void request_dhw_mode(uint8_t target_mode);
    void restore_setpoint_limit(uint8_t ep, uint16_t attr_id, float min_c, float max_c);

    // --- Attributs ZCL (verrou Zigbee tenu par l'appelant) ---
    bool write_attr(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, void* value);
    void publish(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, void* value, float numeric,
                 float min_delta, uint32_t heartbeat_ms = DEFAULT_HEARTBEAT_MS);
    // Second temps de publish() : émet le rapport si la valeur a assez changé ou si son heartbeat est échu
    void report_if_changed(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric,
                           float min_delta, uint32_t heartbeat_ms);
    void set_temp_meas_attr(uint8_t ep, float temp_c);
    void set_binary_input_attr(uint8_t ep, bool state);
    void set_analog_input_attr(uint8_t ep, float val, uint32_t heartbeat_ms = DEFAULT_HEARTBEAT_MS);
    void set_analog_output_attr(uint8_t ep, float val);
    void set_on_off_attr(uint8_t ep, bool on);
    void set_thermostat_temp_attr(uint8_t ep, uint16_t attr_id, float temp_c);
    void set_thermostat_u8_attr(uint8_t ep, uint16_t attr_id, uint8_t val,
                                uint32_t heartbeat_ms = DEFAULT_HEARTBEAT_MS);
    void set_thermostat_u16_attr(uint8_t ep, uint16_t attr_id, uint16_t val,
                                 uint32_t heartbeat_ms = DEFAULT_HEARTBEAT_MS);
    // SystemMode d'un thermostat (EP1, EP2 ou EP6), avec remise à zéro préalable de PIHeatingDemand au
    // passage en Off (la pile refuse ensuite d'écrire cet attribut).
    void     set_thermostat_system_mode(uint8_t ep, uint8_t zcl_mode, bool from_coordinator_write);
    uint8_t& tracked_system_mode(uint8_t ep);

    // --- Reporting vers le coordinateur ---
    bool              send_report(uint8_t ep, uint16_t cluster_id, uint16_t attr_id);
    void              force_report(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric);
    ReportCacheEntry* find_cache_entry(uint8_t ep, uint16_t cluster_id, uint16_t attr_id);
    void              remember_value(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, float numeric);
    void              forget_value(uint8_t ep, uint16_t cluster_id, uint16_t attr_id);
    // Republication différée : la valeur retenue diffère de celle que le coordinateur vient d'écrire.
    // Rapportée depuis le callback d'écriture, elle partirait AVANT la réponse d'écriture, et le
    // coordinateur remettrait ensuite en cache la valeur écrite. Elle part donc de loop_housekeeping().
    void              schedule_republish(uint8_t ep, uint16_t cluster_id, uint16_t attr_id);
    void              flush_pending_republish();

    bool has_ep_hydraulic1() const;
    bool has_ep_hydraulic2() const;
    bool has_ep_flow() const;

    ReportCacheEntry report_cache_[MAX_REPORT_CACHE] = {};
    size_t           report_cache_count_             = 0;

    struct PendingRepublish {
        uint8_t  ep         = 0;
        uint16_t cluster_id = 0;
        uint16_t attr_id    = 0;
    };
    static constexpr size_t MAX_PENDING_REPUBLISH = 8;
    PendingRepublish pending_republish_[MAX_PENDING_REPUBLISH] = {};
    size_t           pending_republish_count_                  = 0;

    uint32_t zb_reports_sent_       = 0;
    uint32_t zb_reports_suppressed_ = 0;
    uint32_t zb_reports_failed_     = 0;
    uint32_t zb_lock_timeouts_      = 0;
    uint32_t zb_attr_errors_        = 0;

    bsb::BsbCapabilities caps_              = {};
    bsb::BsbBus*         bus_               = nullptr;
    bool                 stack_initialized_ = false;
    std::atomic<bool>    connected_{false};
    std::atomic<bool>    awaiting_pairing_{false};
    std::atomic<bool>    ota_in_progress_{false};
    std::atomic<bool>    coordinator_reached_{false};
    // Identification visuelle demandée par le coordinateur (cluster Identify)
    mutable std::atomic<bool> identify_active_{false};
    std::atomic<uint32_t>     identify_deadline_ms_{0}; // Échéance sur l'horloge ms 32 bits

    // Fraîcheur de chaque mesure exposée dans Zigbee (voir publish_unavailable())
    bsb::MeasurementWatch watch_;

    // Dernier état connu de la PAC (bus BSB)
    uint64_t last_bsb_value_ms_ = 0;    // Dernière valeur valide reçue du régulateur (0 = jamais)
    uint64_t last_generator_ms_ = 0;    // Dernière lecture de l'état du générateur (8400/8413/8006/8402/8414)
    bool     bsb_alive_         = false; // L'état du générateur était frais lors du dernier calcul
    float    hp_flow_c_         = 0.0f;  // Départ PAC (8412) et son horodatage (0 = jamais lu)
    float    hp_return_c_       = 0.0f;  // Retour PAC (8410)
    uint64_t hp_flow_ms_        = 0;
    uint64_t hp_return_ms_      = 0;
    // Durée pendant laquelle la garde hydraulique retire la chauffe à une zone en demande : une mise en
    // route le fait pendant une minute environ, seule une situation qui dure est signalée
    bsb::UnfedZoneWatch unfed_watch_;
    // Dernier code statut non répertorié déjà signalé au journal, par zone (0 = aucun en cours)
    uint16_t z1_unlisted_logged_    = 0;
    uint16_t z2_unlisted_logged_    = 0;
    // Entrées de l'état de chauffe déjà lues depuis le démarrage (ou depuis la fin d'un silence du bus)
    bool     comp_state_known_      = false;
    bool     comp_mod_known_        = false;
    bool     z1_status_known_       = false;
    bool     z2_status_known_       = false;
    bool     dhw_status_known_      = false;

    // SystemMode ZCL actuellement écrit dans chaque thermostat (EP1, EP2, EP6)
    uint8_t  zcl_mode_z1_       = 0x04;
    uint8_t  zcl_mode_z2_       = 0x04;
    uint8_t  zcl_mode_dhw_      = 0x04;

    // Demandes différées, traitées par loop_housekeeping() dans la tâche propriétaire du bus
    std::atomic<bool> resync_requested_{false};       // Relecture complète du bus après un (ré)appairage
    bool     dhw_reduced_restore_pending_ = false;    // Republier la consigne réduite ECS d'origine
    // Mémorisation en NVS « cette zone est alimentée par une sonde externe » : -1 = rien à écrire,
    // 0 = effacer, 1 = poser (voir BsbDiscovery::set_external_sensor_used)
    int8_t   ext_used_request_[2] = {-1, -1};
    bool     comp_running_     = false;
    uint8_t  comp_mod_         = 0;
    bool     backup_running_   = false;
    uint8_t  backup_mod_       = 0;
    uint16_t hp_status_code_   = 0;
    bool     hp_pump_running_  = false;
    bool     z1_pump_running_  = false;
    bool     z2_pump_running_  = false;
    uint8_t  z1_mode_          = 1;
    uint8_t  z2_mode_          = 1;
    bool     z1_mode_known_    = false; // Prog 700 relu au moins une fois depuis le démarrage
    bool     z2_mode_known_    = false; // Prog 1000 relu au moins une fois depuis le démarrage
    uint16_t z1_status_code_   = 0;
    uint16_t z2_status_code_   = 0;
    uint16_t dhw_status_code_  = 0;
    uint8_t  dhw_mode_         = 1;
    bool     dhw_mode_known_   = false; // Prog 1600 relu au moins une fois depuis le démarrage

    // État de chauffe publié par thermostat. Une mise en chauffe n'est publiée qu'une fois confirmée
    // (voir bsb::HeatingConfirmation).
    HeatingPublication z1_heating_;
    HeatingPublication z2_heating_;
    HeatingPublication dhw_heating_;

    uint64_t last_housekeeping_ms_ = 0;
    uint64_t last_running_sync_ms_ = 0;
    bool     ext_antenna_enabled_  = false;

    ExternalSensorState ext_sensor_z1_ = {};
    ExternalSensorState ext_sensor_z2_ = {};
};

} // namespace zb_bsb

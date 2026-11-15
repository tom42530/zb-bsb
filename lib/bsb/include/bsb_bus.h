#pragma once

#if defined(ESP_PLATFORM)

#include <cstdint>
#include <cstddef>
#include <functional>
#include <utility>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "bsb_types.h"
#include "bsb_frame.h"

namespace bsb {

/**
 * @brief Gestionnaire matériel et protocolaire du bus BSB (Boiler System Bus).
 *
 * Fonctionnalités clés :
 *  - Pilotage matériel ESP-IDF UART (4800 bauds, 8O1) avec inversion logique (^ 0xFF)
 *  - Détection de porteuse / évitement de collision (CSMA) vis-à-vis du module Cozytouch (0x0A) : rien
 *    n'est émis si un octet est arrivé depuis que le silence du bus a été constaté, ni si un début de
 *    trame est en vol ; la trame part ensuite d'un seul tenant, sans qu'une autre tâche s'intercale
 *    entre deux octets
 *  - Écoute passive (Snooping) : exploite gratuitement les réponses RET destinées à 0x0A
 *  - Requêtes synchrones avec réessais intelligents (pour l'Auto-Discovery au démarrage)
 *  - Ordonnanceur asynchrone avec file d'écriture prioritaire (SET) et scrutation périodique (GET)
 *  - Économie des écritures : un SET qui redemande la valeur lue en dernier sur le régulateur est
 *    retenu le temps d'une relecture, et n'est émis que si le paramètre a réellement changé (le
 *    régulateur mémorise chaque écriture, et une automatisation peut répéter la même consigne sans fin)
 *
 * Modèle de concurrence :
 *  - Le bus appartient à UNE seule tâche (celle qui appelle `loop()`, la tâche Arduino). Toutes les
 *    méthodes sont réservées à cette tâche, SAUF les `enqueue_*` qui sont thread-safe : elles déposent
 *    la trame dans une file FreeRTOS sans jamais bloquer l'appelant (ex : la tâche Zigbee).
 *  - Le callback de valeur est appelé depuis la tâche propriétaire, sans aucun verrou tenu.
 */
class BsbBus {
public:
    using ValueCallback = std::function<void(const DecodedValue&)>;

    struct Config {
        uart_port_t uart_port = UART_NUM_1;
        gpio_num_t  rx_pin    = GPIO_NUM_17;
        gpio_num_t  tx_pin    = GPIO_NUM_16;
        int         baud_rate = 4800;
        uint8_t     src_addr  = 0x42;
        uint8_t     dst_addr  = 0x00;
    };

    BsbBus() = default;
    ~BsbBus();
    BsbBus(const BsbBus&)            = delete;
    BsbBus& operator=(const BsbBus&) = delete;

    /**
     * @brief Initialise le périphérique UART matériel et la file d'écriture.
     */
    bool begin(const Config& cfg);

    /**
     * @brief Définit le callback appelé à chaque nouvelle valeur BSB reçue (poll ou snooping).
     */
    void set_value_callback(ValueCallback cb) { value_cb_ = std::move(cb); }

    // ------------------------------------------------------------------------
    // API réservée à la tâche propriétaire du bus
    // ------------------------------------------------------------------------

    /**
     * @brief Interroge de manière synchrone un champ BSB avec gestion de collisions et réessais.
     *        Idéal pour la phase d'Auto-Discovery au démarrage.
     *
     * @param field_id       Identifiant 32 bits du paramètre BSB
     * @param out_val        Structure remplie avec la valeur décodée
     * @param expected_type  Type attendu (ou ValueType::Unknown pour déduction automatique)
     * @param max_retries    Nombre maximal de tentatives (défaut: 4)
     * @param timeout_ms     Délai d'attente par tentative en ms (défaut: 350 ms)
     * @param out_answer     Si non nul, reçoit la trame RET telle que le régulateur l'a émise (diagnostic :
     *                       charge utile brute d'un paramètre dont le format n'est pas connu)
     * @return true si le régulateur a répondu par une trame RET (false immédiat sur NACK)
     */
    bool query_sync(uint32_t field_id,
                    DecodedValue& out_val,
                    ValueType expected_type = ValueType::Unknown,
                    uint8_t max_retries = 4,
                    uint32_t timeout_ms = 350,
                    BsbFrame* out_answer = nullptr);

    /**
     * @brief Ajoute un paramètre à la liste de scrutation périodique (GET automatique).
     */
    bool register_poll_field(uint32_t field_id, uint32_t interval_ms, ValueType type = ValueType::Unknown);

    /**
     * @brief Vide la table de scrutation périodique.
     */
    void clear_poll_fields();

    /**
     * @brief Avance la prochaine lecture d'un paramètre déjà scruté (rafraîchissement anticipé).
     */
    void request_refresh(uint32_t field_id, uint32_t delay_ms = 400);

    /**
     * @brief Relit dès que possible toute la table de scrutation (resynchronisation complète).
     */
    void refresh_all();

    /**
     * @brief Boucle d'exécution non-bloquante à appeler régulièrement depuis la tâche BSB.
     */
    void loop();

    /**
     * @brief Diagnostic : coupe toute émission et toute écoute pendant `duration_ms`, comme si le câble
     *        du bus était débranché. Sert à vérifier le comportement de la passerelle bus coupé sans
     *        toucher à l'installation. La reprise est automatique à l'échéance.
     */
    void pause(uint32_t duration_ms);

    /**
     * @brief Met fin à une coupure de diagnostic avant son échéance.
     */
    void resume();

    bool paused() const { return paused_; }

    // ------------------------------------------------------------------------
    // API thread-safe (utilisable depuis la tâche Zigbee)
    // ------------------------------------------------------------------------

    /**
     * @brief Enfile une commande d'écriture prioritaire (SET) pour un entier 8 bits / mode.
     * @return false si la file est pleine (la commande est alors perdue)
     */
    bool enqueue_set_uint8(uint32_t field_id, uint8_t value, uint8_t enable_byte = 0x01);

    /**
     * @brief Enfile une commande d'écriture prioritaire (SET) pour une consigne de température (°C).
     */
    bool enqueue_set_temperature(uint32_t field_id, float temp_c, uint8_t enable_byte = 0x01);

    /**
     * @brief Enfile une trame INF de température d'ambiance externe (°C) adressée au régulateur.
     * @param field_id fields::Z1_ROOM_TEMP_INF ou fields::Z2_ROOM_TEMP_INF
     */
    bool enqueue_inf_room_temp(uint32_t field_id, float temp_c);

    /**
     * @brief Statistiques de diagnostic du bus (tâche propriétaire).
     */
    uint32_t tx_count() const { return tx_count_; }
    uint32_t rx_count() const { return rx_count_; }
    /** Trames reçues entières mais rejetées pour CRC faux (collision entre émetteurs, parasite). */
    uint32_t rx_crc_error_count() const { return parser_.crc_error_count(); }
    /** Requêtes d'un autre maître (ex : Cozytouch 0x0A) entendues sur le bus : trafic normal, pas un défaut. */
    uint32_t other_master_request_count() const { return other_master_request_count_; }
    /** Émissions auxquelles nous avons renoncé (porteuse occupée) ou que nous avons interrompues (collision). */
    uint32_t tx_yield_count() const { return tx_yield_count_; }
    /** Trames émises avec un silence de plus de 1 ms entre deux octets (tâche préemptée en cours d'émission). */
    uint32_t tx_stretched_count() const { return tx_stretched_count_; }
    /** Plus long silence laissé entre deux octets d'une de nos trames, en microsecondes. */
    uint32_t tx_max_gap_us() const { return tx_max_gap_us_; }
    uint32_t get_sent_count() const { return get_sent_total_; }
    uint32_t get_answered_count() const { return get_ok_total_; }
    uint8_t  last_reply_percent() const { return last_reply_percent_; }
    uint32_t comp_adjust_count() const { return comp_adjust_count_; }
    uint32_t write_failed_count() const { return write_failed_count_; }
    /** Écritures non émises parce que le régulateur avait déjà la valeur demandée. */
    uint32_t write_skipped_count() const { return write_skipped_count_; }
    int32_t  tx_compensation_us() const { return tx_early_release_us_; }

private:
    static constexpr size_t MAX_POLL_ITEMS  = 32;
    static constexpr size_t MAX_WRITE_QUEUE = 12;
    static constexpr size_t COMP_SLOT_COUNT = 12;

    struct PollItem {
        bool      active       = false;
        uint32_t  field_id     = 0;
        ValueType type         = ValueType::Unknown;
        uint32_t  interval_ms  = 30000;
        uint64_t  next_poll_ms = 0;
        uint8_t   retry_count  = 0;
        // Dernière charge utile RET reçue (octet d'état + valeur) : permet de reconnaître une écriture
        // qui redemande la valeur déjà en place. Longueur 0 = jamais lue, ou trop longue pour être comparée.
        uint8_t   last_payload[4]  = {0};
        uint8_t   last_payload_len = 0;
    };

    struct WriteItem {
        bool     active      = false;
        bool     in_flight   = false; // Trame émise, acquittement attendu pour CETTE version de la trame
        // Écriture retenue : elle redemande la valeur lue en dernier, une relecture décide si elle a lieu
        bool     confirm_first       = false;
        uint64_t confirm_deadline_ms = 0; // Sans réponse à la relecture à cette date, l'écriture part
        BsbFrame frame       = {};
        uint8_t  attempts    = 0;
        uint64_t next_try_ms = 0;
        uint64_t queued_ms   = 0;     // Date de mise en file (une commande trop ancienne est abandonnée)
    };

    static uint64_t now_ms();
    bool       post_frame(const BsbFrame& frame);
    void       drain_inbox();
    void       process_incoming_bytes(BsbFrame* match_frame = nullptr, uint32_t wait_field_id = 0, bool* matched = nullptr);
    void       log_rejected_frame(uint64_t t_now);
    void       handle_received_frame(const BsbFrame& frame);
    void       handle_value_frame(const BsbFrame& frame);
    void       handle_ack_frame(const BsbFrame& frame);
    PollItem*  find_poll_item(uint32_t field_id);
    WriteItem* find_pending_set(uint32_t field_id);
    bool       any_poll_due(uint64_t t_now) const;
    void       issue_get(PollItem& item, uint64_t t_now);
    bool       wait_for_bus_idle(uint32_t max_wait_ms);
    bool       send_frame_raw(const BsbFrame& frame);
    void      comp_on_get_sent();
    void      comp_on_get_answered();

    Config            cfg_                     = {};
    bool              initialized_             = false;
    QueueHandle_t     inbox_                   = nullptr; // Trames SET / INF déposées par les autres tâches
    BsbFrameParser    parser_                  = {};
    ValueCallback     value_cb_                = nullptr;

    uint64_t          last_rx_byte_ms_         = 0;
    uint64_t          bus_busy_until_ms_       = 0;
    uint64_t          last_tx_ms_              = 0;
    uint64_t          rx_frame_since_ms_       = 0;     // Lecture du premier octet de la trame en cours de réception
    bool              report_frame_after_reject_ = false; // Une trame vient d'être rejetée : nommer la suivante

    PollItem          poll_items_[MAX_POLL_ITEMS]  = {};
    size_t            poll_count_                  = 0;
    size_t            poll_cursor_                 = 0;

    WriteItem         write_queue_[MAX_WRITE_QUEUE] = {};
    bool              last_tx_was_write_       = false; // Dernière trame émise : une écriture (SET / INF)

    // Coupure de diagnostic (voir pause())
    bool              paused_                  = false;
    uint64_t          pause_until_ms_          = 0;

    // État de la requête asynchrone en vol
    bool              awaiting_response_       = false;
    uint32_t          awaiting_field_id_       = 0;
    Command           awaiting_cmd_            = Command::None;
    uint64_t          awaiting_deadline_ms_    = 0;

    uint32_t          tx_count_                = 0;
    uint32_t          rx_count_                = 0;
    uint32_t          other_master_request_count_ = 0;
    uint32_t          tx_yield_count_          = 0;
    uint32_t          tx_stretched_count_      = 0;
    uint32_t          tx_max_gap_us_           = 0;
    uint32_t          write_failed_count_      = 0;
    uint32_t          write_skipped_count_     = 0;

    // Pré-compensation TX adaptative (boucle fermée sur le taux de réponse de la PAC)
    int32_t           tx_early_release_us_     = 65;
    int               comp_idx_                = -1;
    int8_t            comp_score_[COMP_SLOT_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    uint8_t           comp_win_sent_           = 0;
    uint8_t           comp_win_ok_             = 0;
    uint8_t           last_reply_percent_      = 100;
    int32_t           calibrated_release_us_   = 65;  // Valeur issue de la calibration au démarrage
    uint8_t           silent_windows_          = 0;   // Fenêtres consécutives sans aucune réponse
    uint32_t          comp_adjust_count_       = 0;
    uint32_t          get_sent_total_          = 0;
    uint32_t          get_ok_total_            = 0;
};

} // namespace bsb

#endif // defined(ESP_PLATFORM)

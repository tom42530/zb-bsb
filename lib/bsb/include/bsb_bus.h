#pragma once

#if defined(ESP_PLATFORM)

#include <cstdint>
#include <cstddef>
#include <functional>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "bsb_types.h"
#include "bsb_frame.h"

namespace bsb {

/**
 * @brief Gestionnaire matériel et protocolaire du bus BSB (Boiler System Bus).
 *
 * Fonctionnalités clés :
 *  - Pilotage matériel ESP-IDF UART (4800 bauds, 8O1) avec inversion logique (^ 0xFF)
 *  - Détection de porteuse / évitement de collision (CSMA) vis-à-vis du module Cozytouch (0x0A)
 *  - Écoute passive (Snooping) : exploite gratuitement les réponses RET destinées à 0x0A
 *  - Requêtes synchrones avec réessais intelligents (pour l'Auto-Discovery au démarrage)
 *  - Ordonnanceur asynchrone avec file d'écriture prioritaire (SET) et scrutation périodique (GET)
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

    /**
     * @brief Initialise le périphérique UART matériel et les verrous FreeRTOS.
     */
    bool begin(const Config& cfg);

    /**
     * @brief Définit le callback appelé à chaque nouvelle valeur BSB reçue (poll ou snooping).
     */
    void set_value_callback(ValueCallback cb) { value_cb_ = cb; }

    /**
     * @brief Interroge de manière synchrone un champ BSB avec gestion de collisions et réessais.
     *        Idéal pour la phase d'Auto-Discovery au démarrage.
     *
     * @param field_id       Identifiant 32 bits du paramètre BSB
     * @param out_val        Structure remplie avec la valeur décodée
     * @param expected_type  Type attendu (ou ValueType::Unknown pour déduction automatique)
     * @param max_retries    Nombre maximal de tentatives (défaut: 4)
     * @param timeout_ms     Délai d'attente par tentative en ms (défaut: 350 ms)
     * @return true si le régulateur a répondu par une trame RET valide
     */
    bool query_sync(uint32_t field_id,
                    DecodedValue& out_val,
                    ValueType expected_type = ValueType::Unknown,
                    uint8_t max_retries = 4,
                    uint32_t timeout_ms = 350);

    /**
     * @brief Ajoute un paramètre à la liste de scrutation périodique (GET automatique).
     */
    bool register_poll_field(uint32_t field_id, uint32_t interval_ms, ValueType type = ValueType::Unknown);

    /**
     * @brief Vide la table de scrutation périodique.
     */
    void clear_poll_fields();

    /**
     * @brief Enfile une commande d'écriture prioritaire (SET) pour un entier 8 bits / mode.
     */
    bool enqueue_set_uint8(uint32_t field_id, uint8_t value, uint8_t enable_byte = 0x01);

    /**
     * @brief Enfile une commande d'écriture prioritaire (SET) pour une consigne de température (°C).
     */
    bool enqueue_set_temperature(uint32_t field_id, float temp_c, uint8_t enable_byte = 0x01);

    /**
     * @brief Enfile une diffusion INF de température d'ambiance externe (°C).
     */
    bool enqueue_inf_room_temp(uint32_t field_id, float temp_c);

    /**
     * @brief Boucle d'exécution non-bloquante à appeler régulièrement depuis la tâche BSB.
     */
    void loop();

    /**
     * @brief Statistiques de diagnostic du bus.
     */
    uint32_t tx_count() const { return tx_count_; }
    uint32_t rx_count() const { return rx_count_; }
    uint32_t crc_error_count() const { return crc_err_count_; }
    uint32_t collision_avoided_count() const { return collision_avoided_count_; }

private:
    static constexpr size_t MAX_POLL_ITEMS  = 32;
    static constexpr size_t MAX_WRITE_QUEUE = 12;

    struct PollItem {
        bool      active       = false;
        uint32_t  field_id     = 0;
        ValueType type         = ValueType::Unknown;
        uint32_t  interval_ms  = 30000;
        uint64_t  next_poll_ms = 0;
        uint8_t   retry_count  = 0;
    };

    struct WriteItem {
        bool     active      = false;
        BsbFrame frame       = {};
        uint8_t  retries     = 0;
        uint64_t next_try_ms = 0;
    };

    static uint64_t now_ms();
    void process_incoming_bytes(BsbFrame* match_frame = nullptr, uint32_t wait_field_id = 0, bool* matched = nullptr);
    void handle_received_frame(const BsbFrame& frame);
    bool wait_for_bus_idle(uint32_t max_wait_ms);
    bool send_frame_raw(const BsbFrame& frame);
    void schedule_quick_refresh(uint32_t field_id, uint32_t delay_ms = 400);

    Config            cfg_                     = {};
    bool              initialized_             = false;
    SemaphoreHandle_t mutex_                   = nullptr;
    BsbFrameParser    parser_                  = {};
    ValueCallback     value_cb_                = nullptr;

    uint64_t          last_rx_byte_ms_         = 0;
    uint64_t          bus_busy_until_ms_       = 0;
    uint64_t          last_tx_ms_              = 0;

    PollItem          poll_items_[MAX_POLL_ITEMS]  = {};
    size_t            poll_count_                  = 0;
    size_t            poll_cursor_                 = 0;

    WriteItem         write_queue_[MAX_WRITE_QUEUE] = {};

    // État de la requête asynchrone en vol
    bool              awaiting_response_       = false;
    uint32_t          awaiting_field_id_       = 0;
    Command           awaiting_cmd_            = Command::None;
    uint64_t          awaiting_deadline_ms_    = 0;

    uint32_t          tx_count_                = 0;
    uint32_t          rx_count_                = 0;
    uint32_t          crc_err_count_           = 0;
    uint32_t          collision_avoided_count_ = 0;
};

} // namespace bsb

#endif // defined(ESP_PLATFORM)

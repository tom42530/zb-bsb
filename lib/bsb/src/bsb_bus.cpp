#if defined(ESP_PLATFORM)

#include <Arduino.h>
#include "bsb_bus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>
#include <cstring>

namespace bsb {

static const char* TAG = "bsb.bus";

// Délai minimal de silence sur le bus avant d'émettre (évite de couper une trame 0x0A)
static constexpr uint64_t BUS_IDLE_SILENCE_MS       = 80;
// Garde supplémentaire lorsqu'un autre maître (ex: 0x0A Cozytouch) échange une rafale GET/RET
static constexpr uint64_t OTHER_MASTER_GUARD_MS     = 190;
// Cadence minimale entre deux requêtes de notre passerelle en régime établi
static constexpr uint64_t MIN_INTER_QUERY_INTERVAL  = 750;
// Timeout d'attente d'un RET ou ACK en mode asynchrone
static constexpr uint64_t ASYNC_RESPONSE_TIMEOUT_MS = 380;
// Nombre maximal d'essais silencieux avant de reporter au prochain cycle
static constexpr uint8_t  MAX_ASYNC_RETRIES         = 4;

BsbBus::~BsbBus() {
    if (initialized_) {
        uart_driver_delete(cfg_.uart_port);
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

uint64_t BsbBus::now_ms() {
    return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
}

int64_t g_tx_early_release_us = 65;

static bool send_wire_bytes_compensated(int tx_pin, int rx_pin, const uint8_t* wire_bytes, size_t len, int64_t early_release_us) {
    // 1. Écoute porteuse (Carrier Sense) pendant 2.5 ms (durée d'un caractère complet 11 bits à 4800 bauds) :
    //    garantit qu'aucun autre maître (0x0A) n'est en train d'émettre un octet.
    const int64_t cs_start = esp_timer_get_time();
    while ((esp_timer_get_time() - cs_start) < 2500) {
        if (digitalRead(rx_pin) == LOW) {
            return false;
        }
    }

    uint8_t bits[BsbFrame::MAX_FRAME_LEN * 11 + 2];
    size_t nbits = 0;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = wire_bytes[i];
        bits[nbits++] = 0; // Start bit (LOW)
        uint8_t ones = 0;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            uint8_t v = (b >> bit) & 0x01;
            bits[nbits++] = v;
            ones += v;
        }
        // Parité IMPAIRE (Odd)
        bits[nbits++] = (ones & 0x01) ? 0 : 1;
        bits[nbits++] = 1; // Stop bit (HIGH)
    }
    bits[nbits] = 1;

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    if (digitalRead(rx_pin) == LOW) {
        portEXIT_CRITICAL(&mux);
        return false;
    }
    const int64_t t_start = esp_timer_get_time();
    for (size_t i = 0; i < nbits; ++i) {
        const int64_t bit_start_us = t_start + static_cast<int64_t>((i * 20833ULL) / 100ULL);
        const int64_t bit_end_us   = t_start + static_cast<int64_t>(((i + 1) * 20833ULL) / 100ULL);

        while (esp_timer_get_time() < bit_start_us) {}
        digitalWrite(tx_pin, bits[i] ? HIGH : LOW);

        if (bits[i] == 0 && bits[i + 1] == 1 && early_release_us > 0) {
            const int64_t release_us = bit_end_us - early_release_us;
            while (esp_timer_get_time() < release_us) {}
            digitalWrite(tx_pin, HIGH);
        } else if (bits[i] == 1) {
            // 2. Détection de collision en cours de bit '1' (CSMA/CD) :
            //    Au milieu d'un bit récessif (HIGH), le bus doit être revenu à HIGH (1).
            //    S'il est à LOW (0), un autre maître (0x0A) parle en même temps : on cède le bus immédiatement.
            const int64_t sample_us = bit_start_us + 135;
            while (esp_timer_get_time() < sample_us) {}
            if (digitalRead(rx_pin) == LOW) {
                digitalWrite(tx_pin, HIGH);
                portEXIT_CRITICAL(&mux);
                return false;
            }
        }
    }
    const int64_t frame_end_us = t_start + static_cast<int64_t>((nbits * 20833ULL) / 100ULL);
    while (esp_timer_get_time() < frame_end_us) {}
    digitalWrite(tx_pin, HIGH);
    portEXIT_CRITICAL(&mux);
    return true;
}

bool BsbBus::begin(const Config& cfg) {
    cfg_ = cfg;
    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
    }

    // --- Auto-calibration 1 µs du temps de stockage optocoupleur (U3 + T1) au démarrage ---
    pinMode(cfg_.tx_pin, OUTPUT);
    digitalWrite(cfg_.tx_pin, HIGH);
    pinMode(cfg_.rx_pin, INPUT_PULLUP);
    delay(20);

    uint64_t wait_idle_start = esp_timer_get_time();
    while ((esp_timer_get_time() - wait_idle_start) < 200000ULL) {
        if (digitalRead(cfg_.rx_pin) == LOW) {
            wait_idle_start = esp_timer_get_time();
        }
        if ((esp_timer_get_time() - wait_idle_start) >= 5000ULL) {
            break;
        }
    }

    const int64_t t0 = esp_timer_get_time();
    digitalWrite(cfg_.tx_pin, LOW);
    int64_t t_fall = -1;
    while ((esp_timer_get_time() - t0) < 208) {
        if (t_fall < 0 && digitalRead(cfg_.rx_pin) == LOW) {
            t_fall = esp_timer_get_time() - t0;
        }
    }
    const int64_t t1 = esp_timer_get_time();
    digitalWrite(cfg_.tx_pin, HIGH);
    int64_t t_rise = -1;
    while ((esp_timer_get_time() - t1) < 2000) {
        if (t_fall < 0 && digitalRead(cfg_.rx_pin) == LOW) {
            t_fall = esp_timer_get_time() - t0;
        }
        if (t_fall >= 0 && digitalRead(cfg_.rx_pin) == HIGH) {
            t_rise = esp_timer_get_time() - t1;
            break;
        }
    }

    if (t_fall >= 0 && t_rise > t_fall) {
        int64_t stretch_us = t_rise - t_fall;
        // On compense ~80% de l'étirement mesuré (ex: 82 us * 0.8 = 65 us), borné entre 0 et 95 us
        int64_t calc_comp = (stretch_us * 80) / 100;
        if (calc_comp < 0) calc_comp = 0;
        if (calc_comp > 95) calc_comp = 95;
        g_tx_early_release_us = calc_comp;
    } else {
        g_tx_early_release_us = 65;
    }

    ESP_LOGI(TAG, "[OSCILLO BSB] Calibration TX : t_ON=%lld us | t_OFF=%lld us -> pre-compensation appliquee = -%lld us",
             static_cast<long long>(t_fall),
             static_cast<long long>(t_rise),
             static_cast<long long>(g_tx_early_release_us));
    delay(15);

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = cfg_.baud_rate;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_ODD;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(cfg_.uart_port, 512, 512, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(cfg_.uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_param_config: %s", esp_err_to_name(err));
        return false;
    }

    // On ne connecte que RX à l'UART1 matériel ; TX (GPIO16) est piloté avec pré-compensation microseconde
    err = uart_set_pin(cfg_.uart_port, UART_PIN_NO_CHANGE, cfg_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_set_pin (TX=%d, RX=%d): %s", cfg_.tx_pin, cfg_.rx_pin, esp_err_to_name(err));
        return false;
    }
    pinMode(cfg_.tx_pin, OUTPUT);
    digitalWrite(cfg_.tx_pin, HIGH);

    // Seuil FIFO RX à 1 octet pour être averti dès le 1er octet (0xDC) émis par 0x0A sur le bus
    uart_set_rx_full_threshold(cfg_.uart_port, 1);
    uart_flush_input(cfg_.uart_port);
    parser_.reset();
    initialized_ = true;

    ESP_LOGI(TAG, "Bus BSB initialise sur UART%d (RX=GPIO%d, TX=GPIO%d, %d 8O1, src=0x%02X -> dst=0x%02X)",
             cfg_.uart_port, cfg_.rx_pin, cfg_.tx_pin, cfg_.baud_rate, cfg_.src_addr, cfg_.dst_addr);
    return true;
}

void BsbBus::process_incoming_bytes(BsbFrame* match_frame, uint32_t wait_field_id, bool* matched) {
    uint8_t buf[64];
    int len = uart_read_bytes(cfg_.uart_port, buf, sizeof(buf), 0);
    if (len <= 0) {
        return;
    }

    const uint64_t t_now = now_ms();
    last_rx_byte_ms_ = t_now;

    for (int i = 0; i < len; ++i) {
        // Inversion logique BSB : les niveaux du bus optocouplé sont inversés bit-à-bit
        const uint8_t logic_byte = static_cast<uint8_t>(buf[i] ^ 0xFF);
        BsbFrame frame;
        if (parser_.feed(logic_byte, frame)) {
            rx_count_++;

            // Ignorer l'écho direct de nos propres trames GET/SET/INF sur le bus half-duplex
            if (frame.src_addr == cfg_.src_addr &&
                (frame.command == Command::Get || frame.command == Command::Set || frame.command == Command::Inf)) {
                continue;
            }

            if (frame.src_addr == 0x00 && frame.dst_addr == cfg_.src_addr) {
                char dbg[96] = {0};
                frame.format_debug(dbg, sizeof(dbg));
                ESP_LOGI(TAG, "[RX PAC ] <- %s (%s)", dbg, get_field_name(frame.field_id));
            }

            // Si un autre maître (ex: 0x0A Cozytouch) échange avec la PAC (GET/SET de 0x0A ou RET vers 0x0A),
            // on réserve le bus pendant 190 ms car le Cozytouch envoie ses requêtes par rafales espacées de ~105 ms.
            if (frame.src_addr != cfg_.src_addr && frame.dst_addr != cfg_.src_addr) {
                bus_busy_until_ms_ = t_now + OTHER_MASTER_GUARD_MS;
                if (frame.command == Command::Get || frame.command == Command::Set) {
                    collision_avoided_count_++;
                }
            }

            handle_received_frame(frame);

            if (matched && match_frame && wait_field_id != 0) {
                const uint32_t swapped_id = BsbFrame::swap_field_msb(wait_field_id);
                if ((frame.command == Command::Ret || frame.command == Command::Ack || frame.command == Command::Nack) &&
                    (frame.field_id == wait_field_id || frame.field_id == swapped_id)) {
                    *match_frame = frame;
                    *matched     = true;
                }
            }
        }
    }
}

void BsbBus::handle_received_frame(const BsbFrame& frame) {
    const uint64_t t_now = now_ms();

    // 1. Gestion des réponses RET et des diffusions INF (y compris Snooping passif de 0x0A !)
    if (frame.command == Command::Ret || frame.command == Command::Inf) {
        ValueType expected = ValueType::Unknown;
        for (size_t i = 0; i < poll_count_; ++i) {
            if (poll_items_[i].active && poll_items_[i].field_id == frame.field_id) {
                expected = poll_items_[i].type;
                // Reporter la prochaine scrutation puisque la valeur vient d'être rafraîchie sur le bus
                poll_items_[i].next_poll_ms = t_now + poll_items_[i].interval_ms;
                poll_items_[i].retry_count  = 0;
                break;
            }
        }

        if (awaiting_response_ && awaiting_cmd_ == Command::Get && awaiting_field_id_ == frame.field_id) {
            awaiting_response_ = false;
        }

        DecodedValue val = frame.decode(expected);
        if (val.valid && value_cb_) {
            value_cb_(val);
        }
    }

    // 2. Gestion des acquittements ACK / NACK suite à nos commandes SET
    if (frame.command == Command::Ack || frame.command == Command::Nack) {
        const uint32_t swapped_id = BsbFrame::swap_field_msb(frame.field_id);
        for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
            if (write_queue_[i].active &&
                (write_queue_[i].frame.field_id == frame.field_id ||
                 write_queue_[i].frame.field_id == swapped_id)) {
                ESP_LOGI(TAG, "Commande SET 0x%08lX acquittee (%s)",
                         static_cast<unsigned long>(write_queue_[i].frame.field_id),
                         frame.command == Command::Ack ? "ACK" : "NACK");
                const uint32_t confirmed_field = write_queue_[i].frame.field_id;
                write_queue_[i].active = false;
                if (awaiting_response_ && awaiting_cmd_ == Command::Set) {
                    awaiting_response_ = false;
                }
                // Relire la valeur 400 ms plus tard pour confirmer l'état dans Zigbee
                schedule_quick_refresh(confirmed_field, 400);

                // Rafraîchir immédiatement la Consigne Active et le Code Statut associés
                if (confirmed_field == fields::Z1_COMFORT_SETPOINT ||
                    confirmed_field == fields::Z1_REDUCED_SETPOINT ||
                    confirmed_field == fields::Z1_MODE) {
                    schedule_quick_refresh(fields::Z1_ACTIVE_SETPOINT, 800);
                    schedule_quick_refresh(fields::Z1_STATUS_CODE, 1200);
                } else if (confirmed_field == fields::Z2_COMFORT_SETPOINT ||
                           confirmed_field == fields::Z2_REDUCED_SETPOINT ||
                           confirmed_field == fields::Z2_MODE) {
                    schedule_quick_refresh(fields::Z2_ACTIVE_SETPOINT, 800);
                    schedule_quick_refresh(fields::Z2_STATUS_CODE, 1200);
                } else if (confirmed_field == fields::DHW_NOMINAL_SETPOINT ||
                           confirmed_field == fields::DHW_REDUCED_SETPOINT ||
                           confirmed_field == fields::DHW_MODE) {
                    schedule_quick_refresh(fields::DHW_STATUS_CODE, 800);
                }
                break;
            }
        }
    }
}

bool BsbBus::wait_for_bus_idle(uint32_t max_wait_ms) {
    const uint64_t start = now_ms();
    while ((now_ms() - start) < max_wait_ms) {
        process_incoming_bytes();
        const uint64_t t_now = now_ms();
        if (t_now >= bus_busy_until_ms_ && (t_now - last_rx_byte_ms_) >= BUS_IDLE_SILENCE_MS) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

bool BsbBus::send_frame_raw(const BsbFrame& frame) {
    if (!initialized_ || frame.raw_len == 0) {
        return false;
    }

    uint8_t wire_bytes[BsbFrame::MAX_FRAME_LEN];
    for (uint8_t i = 0; i < frame.raw_len; ++i) {
        wire_bytes[i] = static_cast<uint8_t>(frame.raw[i] ^ 0xFF);
    }

    if (!send_wire_bytes_compensated(cfg_.tx_pin, cfg_.rx_pin, wire_bytes, frame.raw_len, g_tx_early_release_us)) {
        collision_avoided_count_++;
        bus_busy_until_ms_ = now_ms() + OTHER_MASTER_GUARD_MS;
        return false;
    }

    last_tx_ms_ = now_ms();
    tx_count_++;
    return true;
}

bool BsbBus::query_sync(uint32_t field_id,
                        DecodedValue& out_val,
                        ValueType expected_type,
                        uint8_t max_retries,
                        uint32_t timeout_ms) {
    if (!initialized_) return false;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    const BsbFrame req = BsbFrame::make_get(cfg_.src_addr, cfg_.dst_addr, field_id);

    for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
        // Attendre que le bus soit libre (évite toute collision avec 0x0A Cozytouch)
        wait_for_bus_idle(250);

        send_frame_raw(req);

        BsbFrame resp_frame;
        bool matched = false;
        const uint64_t start_wait = now_ms();

        while ((now_ms() - start_wait) < timeout_ms) {
            process_incoming_bytes(&resp_frame, field_id, &matched);
            if (matched) {
                if (resp_frame.command == Command::Ret) {
                    out_val = resp_frame.decode(expected_type);
                    xSemaphoreGive(mutex_);
                    return true;
                }
                break; // NACK reçu
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Backoff progressif en cas de collision sur le bus
        vTaskDelay(pdMS_TO_TICKS(120 + attempt * 80));
    }

    xSemaphoreGive(mutex_);
    return false;
}

bool BsbBus::register_poll_field(uint32_t field_id, uint32_t interval_ms, ValueType type) {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Vérifier si déjà enregistré
    for (size_t i = 0; i < poll_count_; ++i) {
        if (poll_items_[i].active && poll_items_[i].field_id == field_id) {
            poll_items_[i].interval_ms = interval_ms;
            if (type != ValueType::Unknown) {
                poll_items_[i].type = type;
            }
            xSemaphoreGive(mutex_);
            return true;
        }
    }

    if (poll_count_ >= MAX_POLL_ITEMS) {
        xSemaphoreGive(mutex_);
        return false;
    }

    PollItem& item    = poll_items_[poll_count_];
    item.active       = true;
    item.field_id     = field_id;
    item.type         = (type != ValueType::Unknown) ? type : get_field_value_type(field_id);
    item.interval_ms  = interval_ms;
    // Échelonner les premières lectures au démarrage (150 ms d'écart chacune)
    item.next_poll_ms = now_ms() + (poll_count_ * 150ULL);
    item.retry_count  = 0;
    poll_count_++;

    xSemaphoreGive(mutex_);
    return true;
}

void BsbBus::clear_poll_fields() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (size_t i = 0; i < MAX_POLL_ITEMS; ++i) {
        poll_items_[i] = PollItem{};
    }
    poll_count_  = 0;
    poll_cursor_ = 0;
    xSemaphoreGive(mutex_);
}

void BsbBus::schedule_quick_refresh(uint32_t field_id, uint32_t delay_ms) {
    const uint64_t target = now_ms() + delay_ms;
    for (size_t i = 0; i < poll_count_; ++i) {
        if (poll_items_[i].active && poll_items_[i].field_id == field_id) {
            poll_items_[i].next_poll_ms = target;
            poll_items_[i].retry_count  = 0;
            return;
        }
    }
}

bool BsbBus::enqueue_set_uint8(uint32_t field_id, uint8_t value, uint8_t enable_byte) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    BsbFrame f = BsbFrame::make_set_uint8(cfg_.src_addr, cfg_.dst_addr, field_id, value, enable_byte);

    // Fusionner si une écriture sur le même field_id est déjà en attente
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        if (write_queue_[i].active && write_queue_[i].frame.field_id == field_id) {
            write_queue_[i].frame       = f;
            write_queue_[i].retries     = 0;
            write_queue_[i].next_try_ms = 0;
            xSemaphoreGive(mutex_);
            return true;
        }
    }

    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        if (!write_queue_[i].active) {
            write_queue_[i].active      = true;
            write_queue_[i].frame       = f;
            write_queue_[i].retries     = 0;
            write_queue_[i].next_try_ms = 0;
            xSemaphoreGive(mutex_);
            return true;
        }
    }

    xSemaphoreGive(mutex_);
    return false;
}

bool BsbBus::enqueue_set_temperature(uint32_t field_id, float temp_c, uint8_t enable_byte) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    BsbFrame f = BsbFrame::make_set_temperature(cfg_.src_addr, cfg_.dst_addr, field_id, temp_c, enable_byte);

    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        if (write_queue_[i].active && write_queue_[i].frame.field_id == field_id) {
            write_queue_[i].frame       = f;
            write_queue_[i].retries     = 0;
            write_queue_[i].next_try_ms = 0;
            xSemaphoreGive(mutex_);
            return true;
        }
    }

    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        if (!write_queue_[i].active) {
            write_queue_[i].active      = true;
            write_queue_[i].frame       = f;
            write_queue_[i].retries     = 0;
            write_queue_[i].next_try_ms = 0;
            xSemaphoreGive(mutex_);
            return true;
        }
    }

    xSemaphoreGive(mutex_);
    return false;
}

bool BsbBus::enqueue_inf_room_temp(uint32_t field_id, float temp_c) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    BsbFrame f = BsbFrame::make_inf_room_temperature(cfg_.src_addr, field_id, temp_c);
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        if (!write_queue_[i].active) {
            write_queue_[i].active      = true;
            write_queue_[i].frame       = f;
            write_queue_[i].retries     = MAX_ASYNC_RETRIES - 1; // 1 seul envoi pour un broadcast INF
            write_queue_[i].next_try_ms = 0;
            xSemaphoreGive(mutex_);
            return true;
        }
    }
    xSemaphoreGive(mutex_);
    return false;
}

void BsbBus::loop() {
    if (!initialized_) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // 1. Lire et décoder tous les octets entrants sur l'UART
    process_incoming_bytes();

    const uint64_t t_now = now_ms();

    // 2. Vérifier si une requête en vol a expiré
    if (awaiting_response_ && t_now >= awaiting_deadline_ms_) {
        awaiting_response_ = false;
    }

    // 3. Si une requête est encore en vol ou que le bus n'est pas silencieux, on patiente
    if (awaiting_response_ ||
        t_now < bus_busy_until_ms_ ||
        (t_now - last_rx_byte_ms_) < BUS_IDLE_SILENCE_MS ||
        (t_now - last_tx_ms_) < MIN_INTER_QUERY_INTERVAL) {
        xSemaphoreGive(mutex_);
        return;
    }

    // 4. Priorité absolue aux écritures (SET / INF) en attente dans la file
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        WriteItem& w = write_queue_[i];
        if (w.active && t_now >= w.next_try_ms) {
            if (w.retries >= MAX_ASYNC_RETRIES) {
                ESP_LOGW(TAG, "Abandon SET 0x%08lX apres %u tentatives",
                         static_cast<unsigned long>(w.frame.field_id), w.retries);
                w.active = false;
                continue;
            }

            send_frame_raw(w.frame);
            w.retries++;
            w.next_try_ms = t_now + 600 + (w.retries * 200);

            if (w.frame.command == Command::Inf) {
                w.active = false; // Broadcast INF ne reçoit pas d'ACK
            } else {
                awaiting_response_    = true;
                awaiting_cmd_         = Command::Set;
                awaiting_field_id_    = w.frame.field_id;
                awaiting_deadline_ms_ = t_now + ASYNC_RESPONSE_TIMEOUT_MS;
            }
            xSemaphoreGive(mutex_);
            return;
        }
    }

    // 5. Scrutation périodique (GET) en tourniquet (round-robin)
    if (poll_count_ > 0) {
        for (size_t step = 0; step < poll_count_; ++step) {
            const size_t idx = (poll_cursor_ + step) % poll_count_;
            PollItem& item   = poll_items_[idx];
            if (!item.active || t_now < item.next_poll_ms) {
                continue;
            }

            poll_cursor_ = (idx + 1) % poll_count_;
            BsbFrame get_frame = BsbFrame::make_get(cfg_.src_addr, cfg_.dst_addr, item.field_id);
            send_frame_raw(get_frame);

            item.retry_count++;
            if (item.retry_count >= MAX_ASYNC_RETRIES) {
                // Pas de log d'erreur intempestif : on reporte simplement au cycle suivant
                item.retry_count  = 0;
                item.next_poll_ms = t_now + item.interval_ms;
            } else {
                // En cas de collision ponctuelle, nouvel essai doux dans 2.5s
                item.next_poll_ms = t_now + 2500;
            }

            awaiting_response_    = true;
            awaiting_cmd_         = Command::Get;
            awaiting_field_id_    = item.field_id;
            awaiting_deadline_ms_ = t_now + ASYNC_RESPONSE_TIMEOUT_MS;
            break;
        }
    }

    xSemaphoreGive(mutex_);
}

} // namespace bsb

#endif // defined(ESP_PLATFORM)

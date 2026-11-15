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
// Délai avant de retenter une émission refusée par la détection de porteuse / collision
static constexpr uint64_t TX_BUSY_RETRY_MS          = 120;
// Âge maximal d'une écriture en file : une commande qui n'a pas pu partir (bus saturé ou hors service)
// ne doit pas être exécutée longtemps après que l'utilisateur l'a demandée.
static constexpr uint64_t WRITE_MAX_AGE_MS          = 60000;
// Durée pendant laquelle une écriture redemandant la valeur lue en dernier attend la relecture qui dira
// si elle est utile. Passé ce délai (bus chargé, régulateur muet), elle est émise sans confirmation.
static constexpr uint64_t WRITE_CONFIRM_TIMEOUT_MS  = 4000;

// Candidats de pré-compensation TX (µs) explorés par l'algorithme adaptatif en boucle fermée.
// Le temps de stockage optocoupleur/transistor dérive avec la température : une valeur figée au
// démarrage peut devenir mauvaise après quelques heures et la PAC ignore alors nos trames.
static constexpr uint8_t COMP_CANDIDATES[] = {55, 65, 75, 45, 85, 60, 70, 50, 80, 90, 40, 95};
static constexpr size_t  COMP_COUNT        = sizeof(COMP_CANDIDATES) / sizeof(COMP_CANDIDATES[0]);
// Taille de la fenêtre d'évaluation (requêtes GET) et seuil de taux de réponse acceptable
static constexpr uint8_t COMP_WINDOW       = 12;
static constexpr uint8_t COMP_GOOD_PERCENT = 70;
// Fenêtres consécutives à 0 % de réponse tolérées avant d'incriminer la compensation : un régulateur
// qui redémarre ou une PAC coupée ne répond plus du tout, sans que le réglage d'émission y soit pour rien.
static constexpr uint8_t COMP_SILENT_WINDOWS_BEFORE_ADAPT = 5;
// Attente maximale du silence du bus par lecture synchrone, toutes tentatives confondues : une rafale
// de l'autre maître dure environ 3 s. Au-delà (bus hors tension), on renonce sans émettre.
static constexpr uint32_t SYNC_IDLE_BUDGET_MS   = 4000;
static constexpr uint32_t SYNC_IDLE_MIN_WAIT_MS = 300;

static constexpr int64_t BIT_US_X100 = 20833; // 208.33 µs à 4800 bauds

BsbBus::~BsbBus() {
    if (initialized_) {
        uart_driver_delete(cfg_.uart_port);
    }
    if (inbox_) {
        vQueueDelete(inbox_);
    }
}

uint64_t BsbBus::now_ms() {
    return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
}

// --- Écoute de porteuse (Carrier Sense) avant émission ---------------------------------------------
// Durée pendant laquelle la ligne est observée, interruptions masquées, juste avant notre bit de start.
// Elle sert à voir le premier octet d'une trame en train de passer : sur le fil, l'octet de début 0xDC
// ne laisse jamais la ligne au repos plus de 2 bits de suite (417 µs à 4800 bauds). Une trame plus
// avancée n'a pas besoin d'être écoutée : ses octets déjà reçus la trahissent (voir ci-dessous).
static constexpr int64_t CARRIER_SENSE_US = 700;
// Silence entre deux de nos octets au-delà duquel la trame est comptée comme étirée (diagnostic)
static constexpr int64_t TX_STRETCH_US    = 1000;

/** Ce que l'émission d'une trame a observé de sa propre exécution (diagnostic). */
struct TxReport {
    int64_t max_gap_us = 0; // Plus long silence laissé entre deux octets de la trame
};

static bool send_wire_bytes_compensated(uart_port_t uart_port, int tx_pin, int rx_pin, const uint8_t* wire_bytes,
                                        size_t len, int64_t early_release_us, TxReport& report) {
    if (len == 0) {
        return false;
    }

    // 1. Écoute de porteuse. Tout se passe interruptions masquées, d'un seul tenant avec le premier
    //    octet : la tâche ne peut plus être suspendue entre le moment où le bus est jugé libre et notre
    //    bit de start. (Une écoute interruptions ouvertes ne vaut rien ici : la pile Zigbee reprend la
    //    main à chaque milliseconde, et la ligne n'est pas observée pendant ce temps.)
    //      - Aucun octet ne doit être arrivé depuis la dernière lecture de l'UART par l'appelant, qui a
    //        établi le silence du bus. Si la tâche a été suspendue depuis, une trame a pu commencer, voire
    //        passer en entier : sa suite, ou la réponse qu'elle appelle, est peut-être imminente.
    //      - La ligne doit rester au repos pendant CARRIER_SENSE_US : aucun début de trame n'est en vol.
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    size_t buffered = 0;
    if (uart_get_buffered_data_len(uart_port, &buffered) != ESP_OK || buffered != 0) {
        portEXIT_CRITICAL(&mux);
        return false;
    }
    const int64_t listen_start_us = esp_timer_get_time();
    while ((esp_timer_get_time() - listen_start_us) < CARRIER_SENSE_US) {
        if (digitalRead(rx_pin) == LOW) {
            portEXIT_CRITICAL(&mux);
            return false;
        }
    }

    // 2. Émission octet par octet. Les interruptions ne sont masquées que pendant UN caractère
    //    (~2.3 ms) et non pendant toute la trame (~30 ms) : sur l'ESP32-C6 mono-cœur, masquer les
    //    interruptions 30 ms toutes les 0.8 s rend la radio IEEE 802.15.4 sourde et dégrade le
    //    maillage Zigbee. Entre deux octets la ligne reste au repos (HIGH) : un bit de stop
    //    légèrement allongé est parfaitement conforme au protocole UART.
    //    La section critique du premier octet est déjà ouverte par l'écoute ci-dessus.
    int64_t prev_byte_end_us = 0;
    for (size_t n = 0; n < len; ++n) {
        uint8_t bits[12];
        const uint8_t b = wire_bytes[n];
        uint8_t nb = 0;
        bits[nb++] = 0; // Start bit (LOW)
        uint8_t ones = 0;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint8_t v = (b >> bit) & 0x01;
            bits[nb++] = v;
            ones += v;
        }
        bits[nb++] = (ones & 0x01) ? 0 : 1; // Parité IMPAIRE (Odd)
        bits[nb++] = 1;                     // Stop bit (HIGH)
        bits[nb]   = 1;                     // Sentinelle (repos)

        if (n > 0) {
            portENTER_CRITICAL(&mux);
        }
        // Ligne déjà tirée à LOW avant notre start bit : un autre maître parle -> on cède le bus
        if (digitalRead(rx_pin) == LOW) {
            portEXIT_CRITICAL(&mux);
            return false;
        }
        const int64_t t_start = esp_timer_get_time();
        if (n > 0 && (t_start - prev_byte_end_us) > report.max_gap_us) {
            report.max_gap_us = t_start - prev_byte_end_us;
        }
        for (uint8_t i = 0; i < 11; ++i) {
            const int64_t bit_start_us = t_start + (i * BIT_US_X100) / 100;
            const int64_t bit_end_us   = t_start + ((i + 1) * BIT_US_X100) / 100;

            while (esp_timer_get_time() < bit_start_us) {}
            digitalWrite(tx_pin, bits[i] ? HIGH : LOW);

            if (bits[i] == 0 && bits[i + 1] == 1 && early_release_us > 0) {
                const int64_t release_us = bit_end_us - early_release_us;
                while (esp_timer_get_time() < release_us) {}
                digitalWrite(tx_pin, HIGH);
            } else if (bits[i] == 1) {
                // Détection de collision en cours de bit '1' (CSMA/CD) : au milieu d'un bit récessif,
                // le bus doit être HIGH. S'il est LOW, un autre maître (0x0A) émet : on cède le bus.
                const int64_t sample_us = bit_start_us + 135;
                while (esp_timer_get_time() < sample_us) {}
                if (digitalRead(rx_pin) == LOW) {
                    digitalWrite(tx_pin, HIGH);
                    portEXIT_CRITICAL(&mux);
                    return false;
                }
            }
        }
        const int64_t byte_end_us = t_start + (11 * BIT_US_X100) / 100;
        while (esp_timer_get_time() < byte_end_us) {}
        digitalWrite(tx_pin, HIGH);
        prev_byte_end_us = byte_end_us;
        portEXIT_CRITICAL(&mux);
    }
    return true;
}

void BsbBus::comp_on_get_sent() {
    static_assert(COMP_COUNT == COMP_SLOT_COUNT, "comp_score_ doit couvrir tous les candidats");

    // Évaluer la fenêtre précédente avant d'émettre la requête suivante (son issue est alors connue)
    if (comp_win_sent_ >= COMP_WINDOW) {
        const uint8_t ratio = static_cast<uint8_t>((comp_win_ok_ * 100U) / comp_win_sent_);
        last_reply_percent_ = ratio;
        bool may_adapt = true;
        if (ratio == 0) {
            // Aucune réponse du tout : régulateur muet (redémarrage, PAC coupée) bien plus probablement
            // qu'une compensation devenue fausse. On patiente quelques fenêtres avant d'explorer.
            if (silent_windows_ < 255) {
                silent_windows_++;
            }
            may_adapt = (silent_windows_ >= COMP_SILENT_WINDOWS_BEFORE_ADAPT);
        } else if (silent_windows_ > 0) {
            // Le régulateur répond de nouveau. Si l'exploration avait démarré pendant son silence, les
            // candidats ont été jugés sans raison : retour à la valeur calibrée, à réévaluer d'abord.
            silent_windows_ = 0;
            if (comp_idx_ >= 0) {
                ESP_LOGI(TAG, "[TX ADAPT] Le regulateur repond de nouveau : retour a la compensation calibree (%ld us)",
                         static_cast<long>(calibrated_release_us_));
                comp_idx_            = -1;
                tx_early_release_us_ = calibrated_release_us_;
                for (size_t i = 0; i < COMP_COUNT; ++i) comp_score_[i] = -1;
            }
            may_adapt = false;
        }

        if (may_adapt && comp_idx_ >= 0) {
            comp_score_[comp_idx_] = static_cast<int8_t>(ratio);
        }

        if (may_adapt && ratio < COMP_GOOD_PERCENT) {
            // 1) Explorer d'abord les candidats jamais testés
            int next = -1;
            for (size_t i = 0; i < COMP_COUNT; ++i) {
                if (static_cast<int>(i) != comp_idx_ && comp_score_[i] < 0) { next = static_cast<int>(i); break; }
            }
            // 2) Sinon, revenir au meilleur candidat connu (hors candidat courant)
            if (next < 0) {
                int best_score = -1;
                for (size_t i = 0; i < COMP_COUNT; ++i) {
                    if (static_cast<int>(i) != comp_idx_ && comp_score_[i] > best_score) {
                        best_score = comp_score_[i];
                        next       = static_cast<int>(i);
                    }
                }
                // Tous les candidats sont mauvais : on oublie les scores pour un nouveau balayage complet
                if (best_score < COMP_GOOD_PERCENT) {
                    for (size_t i = 0; i < COMP_COUNT; ++i) comp_score_[i] = -1;
                }
            }
            if (next >= 0) {
                ESP_LOGW(TAG, "[TX ADAPT] Taux de reponse PAC %u%% < %u%% avec compensation %ld us -> essai %u us",
                         ratio, COMP_GOOD_PERCENT, static_cast<long>(tx_early_release_us_),
                         COMP_CANDIDATES[next]);
                comp_idx_            = next;
                tx_early_release_us_ = COMP_CANDIDATES[next];
                comp_adjust_count_++;
            }
        }
        comp_win_sent_ = 0;
        comp_win_ok_   = 0;
    }
    comp_win_sent_++;
    get_sent_total_++;
}

void BsbBus::comp_on_get_answered() {
    comp_win_ok_++;
    get_ok_total_++;
}

bool BsbBus::begin(const Config& cfg) {
    cfg_ = cfg;
    if (!inbox_) {
        inbox_ = xQueueCreate(MAX_WRITE_QUEUE, sizeof(BsbFrame));
        if (!inbox_) {
            ESP_LOGE(TAG, "Echec d'allocation de la file d'ecriture BSB");
            return false;
        }
    }

    // --- Auto-calibration 1 µs du temps de stockage optocoupleur (U3 + T1) au démarrage ---
    pinMode(cfg_.tx_pin, OUTPUT);
    digitalWrite(cfg_.tx_pin, HIGH);
    pinMode(cfg_.rx_pin, INPUT_PULLUP);
    delay(20);

    // Attendre 5 ms de repos (ligne RX haute) avant l'impulsion de calibration, sans jamais dépasser
    // 200 ms au total. L'échéance est une horloge distincte du début du silence : avec une seule
    // variable, une ligne RX basse en permanence (bus non alimenté, débranché ou inversé : la PAC est
    // coupée, l'optocoupleur de réception n'est plus polarisé) bloquait setup() pour toujours.
    const int64_t wait_deadline_us = esp_timer_get_time() + 200000;
    int64_t       idle_since_us    = esp_timer_get_time();
    bool          line_idle        = false;
    while (esp_timer_get_time() < wait_deadline_us) {
        const int64_t now_us = esp_timer_get_time();
        if (digitalRead(cfg_.rx_pin) == LOW) {
            idle_since_us = now_us;
        } else if ((now_us - idle_since_us) >= 5000) {
            line_idle = true;
            break;
        }
    }

    int64_t t_fall = -1;
    int64_t t_rise = -1;
    if (line_idle) {
        const int64_t t0 = esp_timer_get_time();
        digitalWrite(cfg_.tx_pin, LOW);
        while ((esp_timer_get_time() - t0) < 208) {
            if (t_fall < 0 && digitalRead(cfg_.rx_pin) == LOW) {
                t_fall = esp_timer_get_time() - t0;
            }
        }
        const int64_t t1 = esp_timer_get_time();
        digitalWrite(cfg_.tx_pin, HIGH);
        while ((esp_timer_get_time() - t1) < 2000) {
            if (t_fall < 0 && digitalRead(cfg_.rx_pin) == LOW) {
                t_fall = esp_timer_get_time() - t0;
            }
            if (t_fall >= 0 && digitalRead(cfg_.rx_pin) == HIGH) {
                t_rise = esp_timer_get_time() - t1;
                break;
            }
        }
    } else {
        // Aucune impulsion n'est émise : la compensation par défaut est conservée et les émissions
        // échoueront proprement à l'écoute de porteuse tant que le bus ne sera pas revenu.
        ESP_LOGW(TAG, "Bus BSB sans tension ou occupe en permanence (ligne RX basse) : calibration TX ignoree.");
    }

    if (t_fall >= 0 && t_rise > t_fall) {
        int64_t stretch_us = t_rise - t_fall;
        // On compense ~80% de l'étirement mesuré (ex: 82 us * 0.8 = 65 us), borné entre 0 et 95 us
        int64_t calc_comp = (stretch_us * 80) / 100;
        if (calc_comp < 0) calc_comp = 0;
        if (calc_comp > 95) calc_comp = 95;
        tx_early_release_us_ = static_cast<int32_t>(calc_comp);
    } else {
        tx_early_release_us_ = 65;
    }

    calibrated_release_us_ = tx_early_release_us_;
    silent_windows_        = 0;
    comp_idx_ = -1; // Valeur calibrée au démarrage, ajustée ensuite en boucle fermée
    for (size_t i = 0; i < COMP_COUNT; ++i) comp_score_[i] = -1;

    ESP_LOGI(TAG, "[OSCILLO BSB] Calibration TX : t_ON=%lld us | t_OFF=%lld us -> pre-compensation appliquee = -%ld us",
             static_cast<long long>(t_fall),
             static_cast<long long>(t_rise),
             static_cast<long>(tx_early_release_us_));
    delay(15);

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = cfg_.baud_rate;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_ODD;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    // Seule la réception passe par l'UART matériel : pas de tampon TX (l'émission est pilotée à la µs)
    esp_err_t err = uart_driver_install(cfg_.uart_port, 512, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(cfg_.uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_param_config: %s", esp_err_to_name(err));
        uart_driver_delete(cfg_.uart_port);
        return false;
    }

    // On ne connecte que RX à l'UART1 matériel ; TX (GPIO16) est piloté avec pré-compensation microseconde
    err = uart_set_pin(cfg_.uart_port, UART_PIN_NO_CHANGE, cfg_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec uart_set_pin (TX=%d, RX=%d): %s", cfg_.tx_pin, cfg_.rx_pin, esp_err_to_name(err));
        uart_driver_delete(cfg_.uart_port);
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
        if (!parser_.in_frame()) {
            rx_frame_since_ms_ = t_now; // Si une trame commence à cet octet, c'est maintenant qu'elle est lue
        }
        const uint32_t crc_errors_before = parser_.crc_error_count();
        BsbFrame       frame;
        const bool     got_frame = parser_.feed(logic_byte, frame);
        if (parser_.crc_error_count() != crc_errors_before) {
            log_rejected_frame(t_now);
            rx_frame_since_ms_ = t_now;
        }
        if (!got_frame) {
            continue;
        }
        rx_count_++;

        if (report_frame_after_reject_) {
            // Dit qui a parlé juste après la trame rejetée : si celle-ci est arrivée amputée de sa fin,
            // c'est peut-être à cet émetteur-là que le sien a cédé le bus.
            report_frame_after_reject_ = false;
            char after[96] = {0};
            frame.format_debug(after, sizeof(after));
            ESP_LOGI(TAG, "Trame recue apres ce rejet : %s (%s)", after, get_field_name(frame.field_id));
        }

        // Ignorer l'écho direct de nos propres trames GET/SET/INF sur le bus half-duplex
        if (frame.src_addr == cfg_.src_addr &&
            (frame.command == Command::Get || frame.command == Command::Set || frame.command == Command::Inf)) {
            continue;
        }

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
        char dbg[96] = {0};
        frame.format_debug(dbg, sizeof(dbg));
        ESP_LOGD(TAG, "[RX] <- %s (%s)", dbg, get_field_name(frame.field_id));
#endif

        // Si un autre maître (ex: 0x0A Cozytouch) échange avec la PAC (GET/SET de 0x0A ou RET vers 0x0A),
        // on réserve le bus pendant 190 ms car le Cozytouch envoie ses requêtes par rafales espacées de ~105 ms.
        if (frame.src_addr != cfg_.src_addr && frame.dst_addr != cfg_.src_addr) {
            bus_busy_until_ms_ = t_now + OTHER_MASTER_GUARD_MS;
            if (frame.command == Command::Get || frame.command == Command::Set) {
                other_master_request_count_++;
            }
        }

        handle_received_frame(frame);

        // Requête synchrone en attente : une réponse RET (même destinée à un autre maître, la donnée
        // est identique) ou un refus NACK qui nous est adressé clôt l'attente.
        if (matched && match_frame && wait_field_id != 0) {
            const uint32_t swapped_id = BsbFrame::swap_field_msb(wait_field_id);
            const bool same_field = (frame.field_id == wait_field_id || frame.field_id == swapped_id);
            const bool is_refusal = (frame.command == Command::Nack || frame.command == Command::Err) &&
                                    frame.dst_addr == cfg_.src_addr;
            const bool is_answer  = (frame.command == Command::Ret) || is_refusal;
            if (same_field && is_answer) {
                *match_frame = frame;
                *matched     = true;
            }
        }
    }
}

void BsbBus::log_rejected_frame(uint64_t t_now) {
    const BsbFrame& bad = parser_.last_rejected();

    char hex[2 * BsbFrame::MAX_FRAME_LEN + 1] = {0};
    for (uint8_t i = 0; i < bad.raw_len && i < BsbFrame::MAX_FRAME_LEN; ++i) {
        snprintf(hex + 2 * i, 3, "%02X", bad.raw[i]);
    }

    // Les deux durées situent la trame par rapport à notre propre émission : une trame rejetée dont le
    // début est lu après la fin de notre émission est passée sur le bus pendant que nous parlions.
    const unsigned long since_start_ms = static_cast<unsigned long>(t_now - rx_frame_since_ms_);
    if (tx_count_ != 0) {
        ESP_LOGW(TAG, "[CRC ERR] src=0x%02X dst=0x%02X cmd=0x%02X len=%u calc=0x%04X != rx=0x%04X | %s | debut lu il y a %lu ms, notre derniere emission terminee il y a %lu ms",
                 bad.src_addr, bad.dst_addr, static_cast<unsigned>(bad.command), bad.raw_len,
                 parser_.last_rejected_computed_crc(), bad.crc, hex, since_start_ms,
                 static_cast<unsigned long>(t_now - last_tx_ms_));
    } else {
        ESP_LOGW(TAG, "[CRC ERR] src=0x%02X dst=0x%02X cmd=0x%02X len=%u calc=0x%04X != rx=0x%04X | %s | debut lu il y a %lu ms, aucune emission de notre part",
                 bad.src_addr, bad.dst_addr, static_cast<unsigned>(bad.command), bad.raw_len,
                 parser_.last_rejected_computed_crc(), bad.crc, hex, since_start_ms);
    }
    report_frame_after_reject_ = true;
}

BsbBus::PollItem* BsbBus::find_poll_item(uint32_t field_id) {
    for (size_t i = 0; i < poll_count_; ++i) {
        if (poll_items_[i].active && poll_items_[i].field_id == field_id) {
            return &poll_items_[i];
        }
    }
    return nullptr;
}

void BsbBus::handle_received_frame(const BsbFrame& frame) {
    switch (frame.command) {
        case Command::Ret:
            handle_value_frame(frame);
            break;
        case Command::Ack:
        case Command::Nack:
        case Command::Err:
            // ERR (0x08) est la réponse du régulateur à une lecture ou une écriture qu'il refuse
            // (paramètre non pris en charge...) : il se traite comme un NACK.
            // Seuls les acquittements qui nous sont adressés concernent nos propres requêtes :
            // un ACK destiné au Cozytouch ne doit pas solder une de nos écritures en attente.
            if (frame.dst_addr == cfg_.src_addr) {
                handle_ack_frame(frame);
            } else if (frame.command == Command::Ack) {
                // Un autre maître (Cozytouch, afficheur de la PAC) vient de modifier ce paramètre :
                // on le relit sans attendre la prochaine scrutation pour refléter le changement dans Zigbee.
                request_refresh(frame.field_id, 400);
                request_refresh(BsbFrame::swap_field_msb(frame.field_id), 400);
            }
            break;
        default:
            // Les diffusions INF n'ont pas la même structure de charge utile que les réponses RET
            // (pas d'octet d'état en tête) : elles ne sont pas décodées comme des valeurs.
            break;
    }
}

BsbBus::WriteItem* BsbBus::find_pending_set(uint32_t field_id) {
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        WriteItem& w = write_queue_[i];
        if (w.active && w.frame.command == Command::Set && w.frame.field_id == field_id) {
            return &w;
        }
    }
    return nullptr;
}

bool BsbBus::any_poll_due(uint64_t t_now) const {
    for (size_t i = 0; i < poll_count_; ++i) {
        if (poll_items_[i].active && t_now >= poll_items_[i].next_poll_ms) {
            return true;
        }
    }
    return false;
}

void BsbBus::handle_value_frame(const BsbFrame& frame) {
    const uint64_t t_now = now_ms();

    if (awaiting_response_ && awaiting_cmd_ == Command::Get && awaiting_field_id_ == frame.field_id) {
        awaiting_response_ = false;
        if (frame.dst_addr == cfg_.src_addr) {
            comp_on_get_answered();
        }
    }

    if (WriteItem* w = find_pending_set(frame.field_id)) {
        if (!w->confirm_first) {
            // Une écriture de ce paramètre attend encore son acquittement : cette réponse a pu être
            // produite avant que notre SET n'atteigne le régulateur. Elle est ignorée (sinon l'ancienne
            // valeur écraserait la commande de l'utilisateur) ; la relecture qui suit l'acquittement fera foi.
            return;
        }
        // Écriture retenue le temps de cette relecture : elle redemandait la valeur lue en dernier
        w->confirm_first = false;
        if (!w->frame.requests_value_of(frame.payload, frame.payload_len)) {
            // Le paramètre a changé sur le régulateur depuis notre dernière lecture : l'écriture a lieu,
            // et cette valeur-ci n'est pas publiée (elle écraserait la commande de l'utilisateur).
            return;
        }
        w->active = false;
        write_skipped_count_++;
        ESP_LOGI(TAG, "Ecriture 0x%08lX sans objet : le regulateur a deja cette valeur (aucun SET emis)",
                 static_cast<unsigned long>(frame.field_id));
        // La valeur relue poursuit son chemin : elle confirme au pont l'état réel du régulateur
    }

    // Réponse RET, y compris celles destinées à 0x0A (snooping passif)
    ValueType expected = ValueType::Unknown;
    PollItem* item     = find_poll_item(frame.field_id);
    if (item) {
        expected = item->type;
        // Reporter la prochaine scrutation puisque la valeur vient d'être rafraîchie sur le bus
        item->next_poll_ms = t_now + item->interval_ms;
        item->retry_count  = 0;
        // Mémorisée pour reconnaître une écriture qui redemanderait la valeur déjà en place
        item->last_payload_len = 0;
        if (frame.payload_len <= sizeof(item->last_payload)) {
            memcpy(item->last_payload, frame.payload, frame.payload_len);
            item->last_payload_len = frame.payload_len;
        }
    }

    const DecodedValue val = frame.decode(expected);
    ESP_LOGD(TAG, "[DEC] field=0x%08lX (%s) valid=%d fl=%.2f int=%ld bool=%d",
             static_cast<unsigned long>(frame.field_id), get_field_name(frame.field_id),
             val.valid, val.float_val, static_cast<long>(val.int_val), val.bool_val);
    // Pour un paramètre scruté, une réponse sans valeur exploitable (« --- », mesure hors plage) est
    // transmise elle aussi : le pont doit pouvoir marquer la mesure indisponible au lieu de la laisser figée.
    if (value_cb_ && (val.valid || item != nullptr)) {
        value_cb_(val);
    }
}

void BsbBus::handle_ack_frame(const BsbFrame& frame) {
    const uint64_t t_now      = now_ms();
    const uint32_t swapped_id = BsbFrame::swap_field_msb(frame.field_id);
    const bool     is_nack    = (frame.command == Command::Nack || frame.command == Command::Err);

    // 1. Refus (NACK) d'une de nos lectures : le régulateur ne connaît pas ce paramètre
    if (awaiting_response_ && awaiting_cmd_ == Command::Get &&
        (awaiting_field_id_ == frame.field_id || awaiting_field_id_ == swapped_id)) {
        if (is_nack) {
            awaiting_response_ = false;
            comp_on_get_answered(); // Le régulateur nous a entendus : le lien physique est bon
            if (PollItem* item = find_poll_item(awaiting_field_id_)) {
                item->retry_count  = 0;
                item->next_poll_ms = t_now + item->interval_ms;
            }
            if (WriteItem* w = find_pending_set(awaiting_field_id_)) {
                w->confirm_first = false; // Relecture de confirmation refusée : l'écriture part sans elle
            }
            ESP_LOGD(TAG, "Lecture 0x%08lX refusee par le regulateur (NACK / ERR)",
                     static_cast<unsigned long>(awaiting_field_id_));
        }
        return;
    }

    // 2. Acquittement d'une de nos écritures SET
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        WriteItem& w = write_queue_[i];
        if (!w.active || w.frame.command != Command::Set ||
            (w.frame.field_id != frame.field_id && w.frame.field_id != swapped_id)) {
            continue;
        }
        if (!w.in_flight) {
            // Acquittement d'un envoi précédent : la valeur a été remplacée depuis, elle reste à émettre
            return;
        }

        const uint32_t confirmed_field = w.frame.field_id;
        if (is_nack) {
            write_failed_count_++;
            ESP_LOGW(TAG, "Commande SET 0x%08lX refusee par le regulateur (NACK / ERR)",
                     static_cast<unsigned long>(confirmed_field));
        } else {
            ESP_LOGI(TAG, "Commande SET 0x%08lX acquittee (ACK)", static_cast<unsigned long>(confirmed_field));
        }
        w.active    = false;
        w.in_flight = false;
        if (awaiting_response_ && awaiting_cmd_ == Command::Set && awaiting_field_id_ == confirmed_field) {
            awaiting_response_ = false;
        }

        // Relire la valeur 400 ms plus tard pour refléter l'état réel du régulateur dans Zigbee
        request_refresh(confirmed_field, 400);

        // Rafraîchir également la Consigne Active et le Code Statut associés
        if (confirmed_field == fields::Z1_COMFORT_SETPOINT ||
            confirmed_field == fields::Z1_REDUCED_SETPOINT ||
            confirmed_field == fields::Z1_MODE) {
            request_refresh(fields::Z1_ACTIVE_SETPOINT, 800);
            request_refresh(fields::Z1_STATUS_CODE, 1200);
        } else if (confirmed_field == fields::Z2_COMFORT_SETPOINT ||
                   confirmed_field == fields::Z2_REDUCED_SETPOINT ||
                   confirmed_field == fields::Z2_MODE) {
            request_refresh(fields::Z2_ACTIVE_SETPOINT, 800);
            request_refresh(fields::Z2_STATUS_CODE, 1200);
        } else if (confirmed_field == fields::DHW_NOMINAL_SETPOINT ||
                   confirmed_field == fields::DHW_REDUCED_SETPOINT ||
                   confirmed_field == fields::DHW_MODE) {
            request_refresh(fields::DHW_STATUS_CODE, 800);
        }
        return;
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
    if (!initialized_ || frame.raw_len == 0 || frame.raw_len > BsbFrame::MAX_FRAME_LEN) {
        return false;
    }

    uint8_t wire_bytes[BsbFrame::MAX_FRAME_LEN];
    for (uint8_t i = 0; i < frame.raw_len; ++i) {
        wire_bytes[i] = static_cast<uint8_t>(frame.raw[i] ^ 0xFF);
    }

    // Le temps d'une trame (30 ms environ), aucune autre tâche ne prend la main : seules les interruptions
    // sont servies, entre deux octets. Sans cela la pile Zigbee, réveillée à chaque milliseconde, laisse
    // régulièrement plusieurs millisecondes de silence au milieu de nos trames (mesuré sur l'appareil :
    // 4 à 8 % des trames, jusqu'à 4,9 ms, soit plus de deux caractères). Un autre maître peut alors
    // croire le bus libre, et le régulateur la trame terminée.
    TxReport report;
    vTaskSuspendAll();
    const bool sent = send_wire_bytes_compensated(cfg_.uart_port, cfg_.tx_pin, cfg_.rx_pin, wire_bytes, frame.raw_len,
                                                  tx_early_release_us_, report);
    xTaskResumeAll();
    if (report.max_gap_us > TX_STRETCH_US) {
        tx_stretched_count_++;
    }
    if (report.max_gap_us > static_cast<int64_t>(tx_max_gap_us_)) {
        tx_max_gap_us_ = static_cast<uint32_t>(report.max_gap_us);
    }
    if (!sent) {
        tx_yield_count_++;
        bus_busy_until_ms_ = now_ms() + OTHER_MASTER_GUARD_MS;
        return false;
    }

    last_tx_ms_ = now_ms();
    tx_count_++;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    char dbgtx[96] = {0};
    frame.format_debug(dbgtx, sizeof(dbgtx));
    ESP_LOGD(TAG, "[TX] -> %s (%s)", dbgtx, get_field_name(frame.field_id));
#endif
    return true;
}

bool BsbBus::query_sync(uint32_t field_id,
                        DecodedValue& out_val,
                        ValueType expected_type,
                        uint8_t max_retries,
                        uint32_t timeout_ms,
                        BsbFrame* out_answer) {
    if (!initialized_ || paused_) return false;

    const BsbFrame req = BsbFrame::make_get(cfg_.src_addr, cfg_.dst_addr, field_id);

    // Budget d'attente du silence, commun à toutes les tentatives (voir SYNC_IDLE_BUDGET_MS)
    uint32_t idle_budget_ms = SYNC_IDLE_BUDGET_MS;

    for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
        // Ne jamais émettre tant que l'autre maître échange avec le régulateur : notre trame partirait
        // entre sa question et la réponse, et corromprait cette réponse. Si le silence ne vient pas dans
        // le budget imparti, la tentative est consommée sans émission.
        const uint64_t wait_start = now_ms();
        const uint32_t wait_ms    = (idle_budget_ms > SYNC_IDLE_MIN_WAIT_MS) ? idle_budget_ms : SYNC_IDLE_MIN_WAIT_MS;
        const bool     idle       = wait_for_bus_idle(wait_ms);
        const uint64_t waited_ms  = now_ms() - wait_start;
        idle_budget_ms = (waited_ms >= idle_budget_ms) ? 0 : static_cast<uint32_t>(idle_budget_ms - waited_ms);
        if (!idle) {
            continue;
        }

        if (send_frame_raw(req)) {
            BsbFrame resp_frame;
            bool matched = false;
            const uint64_t start_wait = now_ms();

            while ((now_ms() - start_wait) < timeout_ms) {
                process_incoming_bytes(&resp_frame, field_id, &matched);
                if (matched) {
                    if (resp_frame.command == Command::Ret) {
                        out_val = resp_frame.decode(expected_type);
                        if (out_answer != nullptr) {
                            *out_answer = resp_frame;
                        }
                        return true;
                    }
                    return false; // NACK / ERR : paramètre non pris en charge, inutile d'insister
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        // Backoff progressif en cas de collision ou d'absence de réponse
        vTaskDelay(pdMS_TO_TICKS(120 + attempt * 80));
    }

    return false;
}

bool BsbBus::register_poll_field(uint32_t field_id, uint32_t interval_ms, ValueType type) {
    // Déjà enregistré : simple mise à jour de la cadence
    if (PollItem* existing = find_poll_item(field_id)) {
        existing->interval_ms = interval_ms;
        if (type != ValueType::Unknown) {
            existing->type = type;
        }
        return true;
    }

    if (poll_count_ >= MAX_POLL_ITEMS) {
        ESP_LOGW(TAG, "Table de scrutation pleine : champ 0x%08lX ignore", static_cast<unsigned long>(field_id));
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
    return true;
}

void BsbBus::clear_poll_fields() {
    for (size_t i = 0; i < MAX_POLL_ITEMS; ++i) {
        poll_items_[i] = PollItem{};
    }
    poll_count_  = 0;
    poll_cursor_ = 0;
}

void BsbBus::request_refresh(uint32_t field_id, uint32_t delay_ms) {
    if (PollItem* item = find_poll_item(field_id)) {
        item->next_poll_ms = now_ms() + delay_ms;
        item->retry_count  = 0;
    }
}

void BsbBus::refresh_all() {
    const uint64_t t_now = now_ms();
    for (size_t i = 0; i < poll_count_; ++i) {
        // Échéances échelonnées : la cadence réelle reste bornée par l'intervalle minimal entre requêtes
        poll_items_[i].next_poll_ms = t_now + (i * 100ULL);
        poll_items_[i].retry_count  = 0;
    }
}

bool BsbBus::post_frame(const BsbFrame& frame) {
    if (!inbox_) {
        return false;
    }
    return xQueueSend(inbox_, &frame, 0) == pdTRUE;
}

bool BsbBus::enqueue_set_uint8(uint32_t field_id, uint8_t value, uint8_t enable_byte) {
    return post_frame(BsbFrame::make_set_uint8(cfg_.src_addr, cfg_.dst_addr, field_id, value, enable_byte));
}

bool BsbBus::enqueue_set_temperature(uint32_t field_id, float temp_c, uint8_t enable_byte) {
    return post_frame(BsbFrame::make_set_temperature(cfg_.src_addr, cfg_.dst_addr, field_id, temp_c, enable_byte));
}

bool BsbBus::enqueue_inf_room_temp(uint32_t field_id, float temp_c) {
    return post_frame(BsbFrame::make_inf_room_temperature(cfg_.src_addr, cfg_.dst_addr, field_id, temp_c));
}

void BsbBus::drain_inbox() {
    BsbFrame frame;
    while (xQueueReceive(inbox_, &frame, 0) == pdTRUE) {
        // Fusionner avec une écriture en attente sur le même paramètre : seule la dernière valeur compte
        WriteItem* slot   = nullptr;
        bool       merged = false;
        for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
            WriteItem& w = write_queue_[i];
            if (w.active && w.frame.field_id == frame.field_id && w.frame.command == frame.command) {
                slot   = &w;
                merged = true;
                break;
            }
        }
        if (!slot) {
            for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
                if (!write_queue_[i].active) {
                    slot = &write_queue_[i];
                    break;
                }
            }
        }
        if (!slot) {
            write_failed_count_++;
            ESP_LOGW(TAG, "File d'ecriture BSB pleine : commande 0x%08lX perdue",
                     static_cast<unsigned long>(frame.field_id));
            continue;
        }

        // Écriture qui redemande la valeur lue en dernier sur le régulateur (automatisation qui répète sa
        // consigne, interrupteur déjà dans l'état voulu) : elle est retenue le temps d'une relecture et
        // n'aura lieu que si le paramètre a changé entre-temps. Le régulateur mémorise chaque écriture :
        // autant ne pas lui faire réécrire ce qu'il a déjà. Une écriture qui en remplace une autre déjà
        // émise part toujours, car le régulateur a pu appliquer la précédente.
        const bool was_held = merged && slot->confirm_first;
        bool       hold     = false;
        if (frame.command == Command::Set && (!merged || was_held)) {
            const PollItem* item = find_poll_item(frame.field_id);
            hold = item && item->last_payload_len != 0 &&
                   frame.requests_value_of(item->last_payload, item->last_payload_len);
        }

        const uint64_t t_now = now_ms();
        slot->active        = true;
        slot->in_flight     = false;
        slot->frame         = frame;
        slot->attempts      = 0;
        slot->next_try_ms   = 0;
        slot->queued_ms     = t_now;
        slot->confirm_first = hold;
        if (hold && !was_held) {
            // Échéance fixée une seule fois : des écritures identiques répétées ne la repoussent pas
            slot->confirm_deadline_ms = t_now + WRITE_CONFIRM_TIMEOUT_MS;
            request_refresh(frame.field_id, 0);
        }
    }
}

void BsbBus::pause(uint32_t duration_ms) {
    paused_            = true;
    pause_until_ms_    = now_ms() + duration_ms;
    awaiting_response_ = false;
    ESP_LOGW(TAG, "Bus BSB coupe pour diagnostic pendant %lu s : ni emission ni ecoute.",
             static_cast<unsigned long>(duration_ms / 1000UL));
}

void BsbBus::resume() {
    if (!paused_) {
        return;
    }
    paused_ = false;
    if (initialized_) {
        uart_flush_input(cfg_.uart_port);
    }
    parser_.reset();
    last_rx_byte_ms_ = now_ms();
    ESP_LOGW(TAG, "Fin de la coupure de diagnostic : le bus BSB est de nouveau ecoute.");
}

void BsbBus::issue_get(PollItem& item, uint64_t t_now) {
    const BsbFrame get_frame = BsbFrame::make_get(cfg_.src_addr, cfg_.dst_addr, item.field_id);
    if (!send_frame_raw(get_frame)) {
        // Bus occupé par un autre maître : réessayer dès qu'il redevient silencieux
        item.next_poll_ms = t_now + TX_BUSY_RETRY_MS;
        return;
    }

    last_tx_was_write_ = false;
    comp_on_get_sent();

    item.retry_count++;
    if (item.retry_count >= MAX_ASYNC_RETRIES) {
        item.retry_count  = 0;
        item.next_poll_ms = t_now + item.interval_ms;
    } else {
        item.next_poll_ms = t_now + 2500;
    }

    awaiting_response_    = true;
    awaiting_cmd_         = Command::Get;
    awaiting_field_id_    = item.field_id;
    awaiting_deadline_ms_ = t_now + ASYNC_RESPONSE_TIMEOUT_MS;
}

void BsbBus::loop() {
    if (!initialized_) return;

    // 1. Récupérer les écritures déposées par les autres tâches, puis lire les octets entrants
    drain_inbox();
    if (paused_ && now_ms() >= pause_until_ms_) {
        resume();
    }
    if (paused_) {
        // Coupure de diagnostic : ce qui arrive est jeté, comme si le câble était débranché
        uart_flush_input(cfg_.uart_port);
    } else {
        process_incoming_bytes();
    }

    const uint64_t t_now = now_ms();

    // 2. Vérifier si une requête en vol a expiré
    if (awaiting_response_ && t_now >= awaiting_deadline_ms_) {
        awaiting_response_ = false;
    }

    // 2 bis. Écritures restées trop longtemps en file sans pouvoir être émises (bus saturé ou hors service) :
    //        elles sont abandonnées plutôt que d'être appliquées des minutes ou des heures plus tard.
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        WriteItem& w = write_queue_[i];
        if (w.active && (t_now - w.queued_ms) >= WRITE_MAX_AGE_MS) {
            ESP_LOGW(TAG, "Abandon de l'ecriture 0x%08lX : en file depuis plus de %lu s sans pouvoir etre emise",
                     static_cast<unsigned long>(w.frame.field_id),
                     static_cast<unsigned long>(WRITE_MAX_AGE_MS / 1000ULL));
            w.active    = false;
            w.in_flight = false;
            write_failed_count_++;
            request_refresh(w.frame.field_id, 400);
        }
    }

    // 3. Si une requête est encore en vol ou que le bus n'est pas silencieux, on patiente
    if (paused_ ||
        awaiting_response_ ||
        t_now < bus_busy_until_ms_ ||
        (t_now - last_rx_byte_ms_) < BUS_IDLE_SILENCE_MS ||
        (t_now - last_tx_ms_) < MIN_INTER_QUERY_INTERVAL) {
        return;
    }

    // 4. Écritures (SET / INF) en attente : elles passent avant la scrutation, sauf si la dernière trame
    //    émise était déjà une écriture alors qu'une lecture est due. Un flot d'écritures (appareil Zigbee
    //    défaillant, automatisation emballée) ne peut ainsi pas priver la passerelle de ses lectures.
    const bool yield_to_poll = last_tx_was_write_ && any_poll_due(t_now);
    for (size_t i = 0; !yield_to_poll && i < MAX_WRITE_QUEUE; ++i) {
        WriteItem& w = write_queue_[i];
        if (!w.active || t_now < w.next_try_ms) {
            continue;
        }

        if (w.confirm_first) {
            if (t_now < w.confirm_deadline_ms) {
                continue; // La relecture qui dira si cette écriture est utile est en cours (étape 5)
            }
            w.confirm_first = false; // Pas de réponse à la relecture : l'écriture part sans confirmation
        }

        if (w.attempts >= MAX_ASYNC_RETRIES) {
            ESP_LOGW(TAG, "Abandon SET 0x%08lX apres %u tentatives sans acquittement",
                     static_cast<unsigned long>(w.frame.field_id), w.attempts);
            w.active    = false;
            w.in_flight = false;
            write_failed_count_++;
            // Relire la valeur réelle pour que Zigbee n'affiche pas une consigne jamais appliquée
            request_refresh(w.frame.field_id, 400);
            continue;
        }

        if (!send_frame_raw(w.frame)) {
            // Bus occupé : nouvelle tentative dès qu'il redevient silencieux (ne compte pas comme un essai)
            w.next_try_ms = t_now + TX_BUSY_RETRY_MS;
            return;
        }

        last_tx_was_write_ = true;
        if (w.frame.command == Command::Inf) {
            w.active = false; // Une trame INF ne reçoit pas d'acquittement
            return;
        }

        w.attempts++;
        w.in_flight   = true;
        w.next_try_ms = t_now + 600 + (w.attempts * 200);

        awaiting_response_    = true;
        awaiting_cmd_         = Command::Set;
        awaiting_field_id_    = w.frame.field_id;
        awaiting_deadline_ms_ = t_now + ASYNC_RESPONSE_TIMEOUT_MS;
        return;
    }

    // 5. Relecture attendue par une écriture retenue : elle passe avant la scrutation ordinaire, pour
    //    que la commande de l'utilisateur ne patiente pas derrière toute la table.
    for (size_t i = 0; i < MAX_WRITE_QUEUE; ++i) {
        const WriteItem& w = write_queue_[i];
        if (!w.active || !w.confirm_first) {
            continue;
        }
        PollItem* item = find_poll_item(w.frame.field_id);
        if (item && t_now >= item->next_poll_ms) {
            issue_get(*item, t_now);
            return;
        }
    }

    // 6. Scrutation périodique (GET) en tourniquet (round-robin)
    for (size_t step = 0; step < poll_count_; ++step) {
        const size_t idx = (poll_cursor_ + step) % poll_count_;
        PollItem& item   = poll_items_[idx];
        if (!item.active || t_now < item.next_poll_ms) {
            continue;
        }

        poll_cursor_ = (idx + 1) % poll_count_;
        issue_get(item, t_now);
        return;
    }
}

} // namespace bsb

#endif // defined(ESP_PLATFORM)

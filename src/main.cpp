#include <Arduino.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_task_wdt.h"
#include "board_config.h"
#include "bsb_types.h"
#include "bsb_bus.h"
#include "bsb_discovery.h"
#include "zigbee/zb_bridge.h"

static const char* TAG = "zb_bsb.main";

// Version de la disposition Zigbee indépendante de la topologie PAC (à incrémenter si la structure
// des endpoints / clusters codée dans build_endpoints() change entre deux versions du firmware).
static constexpr uint16_t ZB_LAYOUT_VERSION = 4;

// Chien de garde de la boucle principale : redémarrage si loop() reste bloquée plus de 30 s
static constexpr uint32_t LOOP_WATCHDOG_TIMEOUT_MS = 30000;

// Validation différée d'une image OTA : confirmée après 30 s de rattachement stable au réseau Zigbee,
// annulée (retour à l'ancien firmware) si le réseau n'est pas retrouvé au bout de 10 min. Lorsque
// l'appareil n'a plus d'appairage (NVRAM Zigbee purgée par cette version, appareil retiré du réseau),
// aucun réseau n'est à retrouver : l'image est alors validée après 60 s de fonctionnement stable.
// Les deux derniers délais se comptent à partir du démarrage de la pile Zigbee.
static constexpr uint32_t OTA_CONFIRM_STABLE_MS   = 30000;
static constexpr uint32_t OTA_CONFIRM_UNPAIRED_MS = 60000;
static constexpr uint32_t OTA_CONFIRM_TIMEOUT_MS  = 600000;

// Longueur maximale d'une commande de la console série de diagnostic
static constexpr size_t SERIAL_CMD_MAX_LEN = 48;

// Tampon d'émission de la console USB : assez grand pour un diagnostic complet (~2 ko) et le journal
// de démarrage, car l'écriture est non bloquante (ce qui ne tient pas dans le tampon est perdu).
static constexpr size_t SERIAL_TX_BUFFER_SIZE = 8192;

// Période de contrôle du tampon de la console (voir maintain_serial_console())
static constexpr uint32_t SERIAL_STALL_CHECK_MS = 5000;

// Nombre maximal de scans lors d'une découverte : la topologie n'est retenue que si deux scans
// consécutifs concordent (voir discover_topology()).
static constexpr int DISCOVERY_MAX_PASSES = 3;

// Compteurs de démarrage persistants (diagnostic de terrain : l'appareil a-t-il planté depuis son installation ?)
struct BootStats {
    uint32_t boots         = 0; // Nombre total de démarrages
    uint32_t abnormal      = 0; // Dont démarrages après panne (panic, chien de garde, chute de tension)
    uint8_t  last_abnormal = 0; // Cause du dernier démarrage anormal (esp_reset_reason_t), 0 = aucun
};

static bsb::BsbBus          g_bsb_bus;
static bsb::BsbCapabilities g_caps;
static uint32_t             g_last_diag_ms = 0;
static uint32_t             g_btn_press_start_ms = 0;
static bool                 g_btn_pressed = false;
static bool                 g_ota_pending_verify = false;
static bool                 g_zigbee_nvram_purged = false; // L'appairage a été effacé à ce démarrage
static uint32_t             g_zigbee_connected_since_ms = 0;
static bool                 g_zigbee_was_connected = false;
// Horloge de validation d'une image OTA : elle démarre avec la pile Zigbee (voir confirm_ota_image_when_stable)
static bool                 g_ota_clock_running  = false;
static uint32_t             g_ota_clock_start_ms = 0;
static esp_reset_reason_t   g_reset_reason = ESP_RST_UNKNOWN;
static BootStats            g_boot_stats;
static bool                 g_layout_commit_pending = false; // Nouvelle empreinte Zigbee à valider après la purge
static bool                 g_discovery_retry_wanted = false; // Découverte manquée : à retenter dès que la PAC répond

/**
 * @brief Reporte la validation d'une image OTA fraîchement installée.
 *
 * Par défaut, le cœur Arduino marque toute nouvelle image comme valide avant même setup(), ce qui
 * rend le rollback du bootloader inopérant : un firmware incapable de rejoindre le réseau Zigbee
 * resterait en place sans aucun moyen de le remplacer à distance. Ici la validation n'a lieu
 * qu'une fois la liaison Zigbee rétablie (voir confirm_ota_image_when_stable()).
 */
extern "C" bool verifyRollbackLater() {
    return true;
}

/**
 * @brief Profil de secours (non sauvegardé en NVS) utilisé uniquement si l'ESP32-C6
 *        est démarré sur un bureau (USB) sans être encore raccordé au bus BSB de la PAC.
 *        Au prochain démarrage raccordé à la PAC, l'Auto-Discovery réel s'exécutera.
 */
static void load_bench_fallback_capabilities(bsb::BsbCapabilities& caps) {
    caps = bsb::BsbCapabilities{};
    strncpy(caps.controller_model, "RVS21.831F/127", sizeof(caps.controller_model) - 1);
    caps.system_type           = 20;
    caps.has_outdoor_temp      = true;
    caps.has_hp_flow_temp      = true;
    caps.has_hp_return_temp    = true;
    caps.has_compressor_state  = true;
    caps.has_compressor_mod    = true;
    caps.has_hp_pump_state     = true;
    caps.has_backup_heater     = true;
    caps.has_backup_mod        = true;
    caps.has_hp_status_code    = true;

    caps.zone1.present         = true;
    caps.zone1.has_room_sensor = false; // Filtré par défaut sur Zone 1 Radiateurs
    caps.zone1.has_flow_sensor = false;
    caps.zone1.has_pump_state  = true;
    caps.zone1.has_status_code = true;

    caps.zone2.present         = true;
    caps.zone2.has_room_sensor = true;
    caps.zone2.has_flow_sensor = true;
    caps.zone2.has_pump_state  = true;
    caps.zone2.has_status_code = true;

    caps.has_dhw               = true;
    caps.has_dhw_tank_temp     = true;
    caps.has_dhw_status_code   = true;
}

static void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Partition NVS pleine ou nouvelle version, formatage...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static const char* reset_reason_text(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:    return "mise sous tension";
        case ESP_RST_EXT:        return "broche reset";
        case ESP_RST_SW:         return "redemarrage logiciel";
        case ESP_RST_PANIC:      return "PANNE LOGICIELLE";
        case ESP_RST_INT_WDT:    return "CHIEN DE GARDE (interruptions)";
        case ESP_RST_TASK_WDT:   return "CHIEN DE GARDE (tache bloquee)";
        case ESP_RST_WDT:        return "CHIEN DE GARDE";
        case ESP_RST_DEEPSLEEP:  return "reveil";
        case ESP_RST_BROWNOUT:   return "CHUTE DE TENSION";
        case ESP_RST_USB:        return "reset par USB";
        case ESP_RST_JTAG:       return "reset par JTAG";
        case ESP_RST_PWR_GLITCH: return "PERTURBATION D'ALIMENTATION";
        case ESP_RST_CPU_LOCKUP: return "BLOCAGE DU PROCESSEUR";
        default:                 return "inconnue";
    }
}

static bool is_abnormal_reset(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
        case ESP_RST_BROWNOUT:
        case ESP_RST_PWR_GLITCH:
        case ESP_RST_CPU_LOCKUP:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Relève la cause du démarrage et tient à jour les compteurs persistants.
 *
 * Sans console branchée, la trace d'une panne est perdue : ces compteurs (affichés par `status` et
 * dans le journal périodique) permettent de savoir après coup si l'appareil a redémarré anormalement.
 */
static void record_boot() {
    g_reset_reason = esp_reset_reason();

    nvs_handle_t nvs_h = 0;
    if (nvs_open("zb_bsb_diag", NVS_READWRITE, &nvs_h) != ESP_OK) {
        return;
    }
    nvs_get_u32(nvs_h, "boots", &g_boot_stats.boots);
    nvs_get_u32(nvs_h, "bad_boots", &g_boot_stats.abnormal);
    nvs_get_u8(nvs_h, "last_bad", &g_boot_stats.last_abnormal);

    ++g_boot_stats.boots;
    nvs_set_u32(nvs_h, "boots", g_boot_stats.boots);
    if (is_abnormal_reset(g_reset_reason)) {
        ++g_boot_stats.abnormal;
        g_boot_stats.last_abnormal = static_cast<uint8_t>(g_reset_reason);
        nvs_set_u32(nvs_h, "bad_boots", g_boot_stats.abnormal);
        nvs_set_u8(nvs_h, "last_bad", g_boot_stats.last_abnormal);
    }
    nvs_commit(nvs_h);
    nvs_close(nvs_h);
}

/**
 * @brief Découverte fiabilisée de la topologie de la PAC.
 *
 * La topologie est mémorisée durablement et fixe la structure Zigbee : un organe manqué à cause d'une
 * perturbation passagère du bus (rafale d'un autre maître, régulateur en cours de démarrage) ferait
 * disparaître des entités et imposerait un ré-appairage. Elle n'est donc retenue que si deux scans
 * consécutifs aboutissent à la même structure.
 *
 * @return true si une topologie concordante a été obtenue ; false si le régulateur ne répond pas
 *         ou si les scans ne s'accordent pas (nouvelle tentative au prochain démarrage).
 */
static bool discover_topology(bsb::BsbCapabilities& out_caps) {
    bsb::BsbCapabilities previous;
    bool have_previous = false;

    for (int pass = 1; pass <= DISCOVERY_MAX_PASSES; ++pass) {
        bsb::BsbCapabilities scanned;
        if (!bsb::BsbDiscovery::scan_heat_pump(g_bsb_bus, scanned)) {
            return false; // Régulateur injoignable : inutile d'insister (appareil sur un banc d'essai)
        }
        if (have_previous &&
            zb_bsb::ZbBridge::layout_signature(previous) == zb_bsb::ZbBridge::layout_signature(scanned)) {
            out_caps = scanned;
            return true;
        }
        if (have_previous) {
            ESP_LOGW(TAG, "Decouverte : les scans %d et %d ne concordent pas, nouveau scan...", pass - 1, pass);
        }
        previous      = scanned;
        have_previous = true;
    }

    ESP_LOGE(TAG, "Decouverte instable apres %d scans : topologie non memorisee, nouvelle tentative au prochain demarrage.",
             DISCOVERY_MAX_PASSES);
    return false;
}

/**
 * @brief Détermine si la NVRAM Zigbee doit être purgée avant le démarrage de la pile.
 *
 * Les tables de reporting et de binding mémorisées par ZBOSS référencent les clusters exposés lors
 * de l'appairage. Si la disposition change (nouvelle version du firmware, ou topologie PAC différente
 * après une re-découverte), ces entrées périmées peuvent faire planter la pile : on repart alors
 * d'une NVRAM vierge, ce qui impose un nouvel appairage.
 */
static bool zigbee_layout_changed(const bsb::BsbCapabilities& caps) {
    nvs_handle_t nvs_h = 0;
    if (nvs_open("bsb_topo", NVS_READWRITE, &nvs_h) != ESP_OK) {
        return false;
    }

    const uint32_t current_sig = zb_bsb::ZbBridge::layout_signature(caps);
    uint16_t saved_ver     = 0;
    uint32_t saved_sig     = 0;
    uint8_t  purge_pending = 0;
    const bool has_ver = (nvs_get_u16(nvs_h, "zb_layout_ver", &saved_ver) == ESP_OK);
    const bool has_sig = (nvs_get_u32(nvs_h, "zb_layout_sig", &saved_sig) == ESP_OK);
    nvs_get_u8(nvs_h, "zb_purge_pend", &purge_pending);

    bool changed = false;
    if (purge_pending != 0) {
        ESP_LOGW(TAG, "Purge de la NVRAM Zigbee interrompue au demarrage precedent : elle est refaite.");
        changed = true;
    } else if (!has_ver || saved_ver != ZB_LAYOUT_VERSION) {
        ESP_LOGW(TAG, "Nouvelle disposition Zigbee (v%u) : purge automatique de la NVRAM Zigbee...",
                 static_cast<unsigned>(ZB_LAYOUT_VERSION));
        changed = true;
    } else if (has_sig && saved_sig != current_sig) {
        ESP_LOGW(TAG, "Topologie PAC modifiee (0x%08lX -> 0x%08lX) : purge de la NVRAM Zigbee, re-appairage necessaire.",
                 static_cast<unsigned long>(saved_sig), static_cast<unsigned long>(current_sig));
        changed = true;
    }
    if (changed) {
        // La purge n'a lieu qu'au démarrage de la pile, plus tard dans setup(). Tant qu'elle n'est pas
        // faite, seule la demande est enregistrée : si l'appareil s'arrête entre-temps (coupure, panne au
        // démarrage de la radio), elle sera refaite. La nouvelle empreinte est validée après coup par
        // commit_zigbee_layout().
        if (purge_pending == 0) {
            nvs_set_u8(nvs_h, "zb_purge_pend", 1);
            nvs_commit(nvs_h);
        }
    } else if (!has_sig) {
        // Empreinte absente (firmware antérieur) : elle est simplement adoptée, sans purge.
        nvs_set_u16(nvs_h, "zb_layout_ver", ZB_LAYOUT_VERSION);
        nvs_set_u32(nvs_h, "zb_layout_sig", current_sig);
        nvs_commit(nvs_h);
    }
    nvs_close(nvs_h);
    return changed;
}

/**
 * @brief Valide la nouvelle empreinte de disposition une fois la pile Zigbee démarrée (purge faite).
 */
static void commit_zigbee_layout(const bsb::BsbCapabilities& caps) {
    nvs_handle_t nvs_h = 0;
    if (nvs_open("bsb_topo", NVS_READWRITE, &nvs_h) != ESP_OK) {
        return;
    }
    nvs_set_u16(nvs_h, "zb_layout_ver", ZB_LAYOUT_VERSION);
    nvs_set_u32(nvs_h, "zb_layout_sig", zb_bsb::ZbBridge::layout_signature(caps));
    nvs_erase_key(nvs_h, "zb_purge_pend");
    nvs_commit(nvs_h);
    nvs_close(nvs_h);
    ESP_LOGI(TAG, "NVRAM Zigbee purgee : nouvelle disposition enregistree.");
}

/**
 * @brief Valide l'image OTA en cours d'essai une fois le réseau Zigbee retrouvé, ou revient à
 *        l'ancien firmware si la nouvelle image ne parvient pas à rejoindre le réseau.
 */
static void confirm_ota_image_when_stable() {
    if (!g_ota_pending_verify) {
        return;
    }
    const uint32_t now = millis();
    zb_bsb::ZbBridge& bridge = zb_bsb::ZbBridge::instance();
    // « Connecté » ne suffit pas : un routeur déjà appairé retrouve son réseau en relisant sa mémoire,
    // sans émettre. On exige un aller-retour radio constaté avec le coordinateur.
    const bool connected = bridge.is_connected() && bridge.coordinator_reached();
    // Sans appairage, il n'y a aucun réseau à retrouver : exiger une connexion ferait revenir à tort
    // à l'ancien firmware (qui se retrouverait lui-même sans appairage et se verrait reproposer l'image).
    const bool unpaired = g_zigbee_nvram_purged || bridge.is_awaiting_pairing();

    // Les délais se comptent à partir du démarrage de la pile Zigbee, pas de la mise sous tension : une
    // découverte de la PAC peut retenir setup() plus d'une minute, et l'image serait sinon validée dès
    // le premier passage ici, sans avoir fait ses preuves. Tant qu'elle n'est pas validée, un redémarrage
    // (panne au lancement de la radio) fait revenir le chargeur d'amorçage à l'image précédente.
    if (!g_ota_clock_running && bridge.is_stack_started()) {
        g_ota_clock_running  = true;
        g_ota_clock_start_ms = now;
    }
    const uint32_t since_stack_ms = g_ota_clock_running ? (now - g_ota_clock_start_ms) : 0;
    // Pile jamais démarrée : cette image ne sait pas lancer la radio, le délai court depuis la mise sous tension
    const bool stack_never_started = !g_ota_clock_running && now >= OTA_CONFIRM_TIMEOUT_MS;

    if (connected && !g_zigbee_was_connected) {
        g_zigbee_connected_since_ms = now;
    }
    g_zigbee_was_connected = connected;

    if (connected && (now - g_zigbee_connected_since_ms) >= OTA_CONFIRM_STABLE_MS) {
        g_ota_pending_verify = false;
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            ESP_LOGI(TAG, "[OTA] Nouveau firmware valide (reseau Zigbee retrouve) : rollback annule.");
        }
    } else if (!connected && unpaired && g_ota_clock_running && since_stack_ms >= OTA_CONFIRM_UNPAIRED_MS) {
        g_ota_pending_verify = false;
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            ESP_LOGI(TAG, "[OTA] Nouveau firmware valide (appareil sans appairage Zigbee, en attente d'un reseau).");
        }
    } else if (!connected &&
               (stack_never_started ||
                (!unpaired && g_ota_clock_running && since_stack_ms >= OTA_CONFIRM_TIMEOUT_MS))) {
        g_ota_pending_verify = false; // Une seule tentative : en cas d'échec du rollback, on continue sur cette image
        ESP_LOGE(TAG, "[OTA] Reseau Zigbee non retrouve apres %lu min avec le nouveau firmware : retour a la version precedente.",
                 static_cast<unsigned long>(OTA_CONFIRM_TIMEOUT_MS / 60000UL));
        delay(200);
        const esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot(); // Ne revient pas en cas de succès
        ESP_LOGE(TAG, "[OTA] Rollback impossible (%s) : le firmware courant est conserve.", esp_err_to_name(err));
    }
}

static void check_boot_button() {
    if (zb_bsb::ZbBridge::instance().is_ota_in_progress()) {
        // Bouton ignoré pendant une mise à jour. L'appui en cours est oublié : sinon, à la fin du
        // transfert, sa durée engloberait tout l'OTA et déclencherait un reset usine intempestif.
        g_btn_pressed = false;
        return;
    }
    const bool pressed_now = (gpio_get_level(zb_bsb::board::PIN_BOOT_BUTTON) == 0);
    const uint32_t now = millis();

    if (pressed_now && !g_btn_pressed) {
        g_btn_pressed = true;
        g_btn_press_start_ms = now;
    } else if (pressed_now && g_btn_pressed) {
        const uint32_t held_ms = now - g_btn_press_start_ms;
        if (held_ms >= 3000 && held_ms < 8000) {
            // Clignotement rapide (120 ms) : indique qu'un relâchement lancera une Re-Découverte BSB
            zb_bsb::board::set_status_led(((now / 120) % 2) == 0);
        } else if (held_ms >= 8000) {
            // Clignotement stroboscopique (50 ms) : alerte Reset Usine Zigbee + BSB
            zb_bsb::board::set_status_led(((now / 50) % 2) == 0);
        }
    } else if (!pressed_now && g_btn_pressed) {
        const uint32_t held_ms = now - g_btn_press_start_ms;
        g_btn_pressed = false;

        if (held_ms >= 8000) {
            ESP_LOGW(TAG, "Appui tres long (>8s) : Reset Usine Zigbee + Effacement topologie BSB...");
            zb_bsb::board::set_status_led(false);
            zb_bsb::ZbBridge::instance().set_external_antenna_enabled(false);
            bsb::BsbDiscovery::clear_nvs();
            bsb::BsbDiscovery::clear_rescan_request();
            bsb::BsbDiscovery::set_external_sensor_used(1, false);
            bsb::BsbDiscovery::set_external_sensor_used(2, false);
            zb_bsb::ZbBridge::instance().factory_reset(); // Efface zb_storage puis redémarre
            delay(500);
            esp_restart();
        } else if (held_ms >= 3000) {
            ESP_LOGW(TAG, "Appui long (3-8s) : Relance de l'Auto-Decouverte PAC BSB...");
            zb_bsb::board::set_status_led(false);
            // La topologie en place est conservée comme repli : elle n'est remplacée qu'après une
            // découverte réussie (PAC coupée à ce moment-là = l'appareil continue comme avant).
            bsb::BsbDiscovery::request_rescan();
            delay(300);
            esp_restart();
        }
    }
}

static void update_status_led() {
    if (g_btn_pressed) {
        // La LED est pilotée par check_boot_button() lors des appuis sur le bouton BOOT
        return;
    }

    const uint32_t now = millis();
    zb_bsb::ZbBridge& bridge = zb_bsb::ZbBridge::instance();
    if (bridge.is_identifying()) {
        // Identification demandée depuis Home Assistant : clignotement stroboscopique (50 ms)
        zb_bsb::board::set_status_led(((now / 50) % 2) == 0);
    } else if (bridge.is_ota_in_progress()) {
        // Mise à jour OTA en cours : clignotement soutenu (200 ms)
        zb_bsb::board::set_status_led(((now / 200) % 2) == 0);
    } else if (bridge.is_connected()) {
        // Associé et connecté au réseau Zigbee : LED allumée fixe
        zb_bsb::board::set_status_led(true);
    } else {
        // En recherche / appairage Zigbee : clignotement régulier (500 ms)
        zb_bsb::board::set_status_led(((now / 500) % 2) == 0);
    }
}

static void log_bus_health() {
    zb_bsb::ZbBridge& bridge = zb_bsb::ZbBridge::instance();
    ESP_LOGI(TAG, "[ Sante Bus BSB ] TX=%lu | RX=%lu (%lu rejetees) | Reponses PAC=%lu/%lu (fenetre %u%%, comp=%ld us, %lu ajust.) | Requetes autres maitres=%lu | Emissions cedees=%lu (%lu trames etirees, max %lu us) | Ecritures echouees=%lu (%lu sans objet) | Zigbee=%s (Reps: %lu emis, %lu suppr, %lu err, %lu lock) | Heap=%lu (min %lu) | Actif depuis %lu s (demarrage n.%lu : %s ; %lu anormal(aux))%s",
             static_cast<unsigned long>(g_bsb_bus.tx_count()),
             static_cast<unsigned long>(g_bsb_bus.rx_count()),
             static_cast<unsigned long>(g_bsb_bus.rx_crc_error_count()),
             static_cast<unsigned long>(g_bsb_bus.get_answered_count()),
             static_cast<unsigned long>(g_bsb_bus.get_sent_count()),
             static_cast<unsigned>(g_bsb_bus.last_reply_percent()),
             static_cast<long>(g_bsb_bus.tx_compensation_us()),
             static_cast<unsigned long>(g_bsb_bus.comp_adjust_count()),
             static_cast<unsigned long>(g_bsb_bus.other_master_request_count()),
             static_cast<unsigned long>(g_bsb_bus.tx_yield_count()),
             static_cast<unsigned long>(g_bsb_bus.tx_stretched_count()),
             static_cast<unsigned long>(g_bsb_bus.tx_max_gap_us()),
             static_cast<unsigned long>(g_bsb_bus.write_failed_count()),
             static_cast<unsigned long>(g_bsb_bus.write_skipped_count()),
             bridge.is_connected() ? "CONNECTE" : "EN RECHERCHE",
             static_cast<unsigned long>(bridge.reports_sent()),
             static_cast<unsigned long>(bridge.reports_suppressed()),
             static_cast<unsigned long>(bridge.reports_failed()),
             static_cast<unsigned long>(bridge.lock_timeouts()),
             static_cast<unsigned long>(esp_get_free_heap_size()),
             static_cast<unsigned long>(esp_get_minimum_free_heap_size()),
             static_cast<unsigned long>(millis() / 1000UL),
             static_cast<unsigned long>(g_boot_stats.boots),
             reset_reason_text(g_reset_reason),
             static_cast<unsigned long>(g_boot_stats.abnormal),
             g_bsb_bus.paused() ? " | BUS COUPE POUR DIAGNOSTIC" : "");
}

/**
 * @brief Analyse stricte d'un identifiant de champ BSB en hexadécimal (1 à 8 chiffres, préfixe 0x toléré).
 */
static bool parse_field_id(String text, uint32_t& out) {
    if (text.startsWith("0x") || text.startsWith("0X")) {
        text.remove(0, 2);
    }
    if (text.length() == 0 || text.length() > 8) {
        return false;
    }
    for (size_t i = 0; i < text.length(); ++i) {
        if (!isHexadecimalDigit(text[i])) {
            return false;
        }
    }
    out = strtoul(text.c_str(), nullptr, 16);
    return out != 0;
}

/**
 * @brief Analyse stricte d'un entier décimal 0..255 (chiffres uniquement).
 */
static bool parse_uint8(const String& text, uint8_t& out) {
    if (text.length() == 0 || text.length() > 3) {
        return false;
    }
    for (size_t i = 0; i < text.length(); ++i) {
        if (!isDigit(text[i])) {
            return false;
        }
    }
    const long value = text.toInt();
    if (value > 255) {
        return false;
    }
    out = static_cast<uint8_t>(value);
    return true;
}

/**
 * @brief Affiche la réponse du régulateur à une lecture brute : ce qu'elle vaut selon sa longueur (un octet
 *        d'état suivi d'une valeur sur 1, 2 ou 4 octets), puis sa charge utile en hexadécimal.
 *
 * L'octet d'état dit si la fonction existe : bit 0 à 1, le régulateur affiche « --- » et les octets qui
 * suivent ne sont que du remplissage.
 */
static void print_raw_answer(uint32_t fid, const bsb::BsbFrame& answer) {
    const unsigned long id = static_cast<unsigned long>(fid);
    if (answer.payload_len == 0) {
        Serial.printf("GET 0x%08lX -> reponse sans charge utile\r\n", id);
        return;
    }

    char   hex[3 * bsb::BsbFrame::MAX_PAYLOAD_LEN + 1] = {0};
    size_t pos = 0;
    for (uint8_t i = 0; i < answer.payload_len && (pos + 4) <= sizeof(hex); ++i) {
        pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", answer.payload[i]));
    }

    if ((answer.payload[0] & 0x01) != 0) {
        Serial.printf("GET 0x%08lX -> \"---\" : fonction absente ou desactivee | octets : %s\r\n", id, hex);
        return;
    }

    const uint8_t* value = &answer.payload[1];
    switch (answer.payload_len) {
        case 2:
            Serial.printf("GET 0x%08lX -> brut=%u | octets : %s\r\n", id, value[0], hex);
            break;
        case 3: {
            const int16_t word = static_cast<int16_t>((static_cast<uint16_t>(value[0]) << 8) | value[1]);
            Serial.printf("GET 0x%08lX -> brut=%d (%.2f C si temperature) | octets : %s\r\n", id, word,
                          static_cast<double>(word) / 64.0, hex);
            break;
        }
        case 5: {
            const uint32_t dword = (static_cast<uint32_t>(value[0]) << 24) | (static_cast<uint32_t>(value[1]) << 16) |
                                   (static_cast<uint32_t>(value[2]) << 8) | value[3];
            Serial.printf("GET 0x%08lX -> brut=%lu (valeur sur 32 bits) | octets : %s\r\n", id,
                          static_cast<unsigned long>(dword), hex);
            break;
        }
        default:
            Serial.printf("GET 0x%08lX -> format non interprete (%u octets) | octets : %s\r\n", id, answer.payload_len,
                          hex);
            break;
    }
}

/**
 * @brief Exécute une commande de la console série de diagnostic (USB CDC).
 *
 *   status | diag          : état détaillé du pont
 *   report                 : ré-émission complète des attributs Zigbee
 *   get <field_hex>        : lecture brute d'un paramètre BSB
 *   set <field_hex> <val>  : écriture d'un paramètre BSB 8 bits (diagnostic avancé)
 *   dhw on|off|<mode>      : mode ECS (Prog 1600)
 *   bus off [min] | bus on : coupe le bus (ni émission ni écoute) pendant 1 à 60 min, 10 par défaut, pour
 *                            vérifier le comportement de la passerelle bus débranché ; reprise automatique
 *   antenna int|ext        : choix de l'antenne Zigbee
 *   rescan                 : efface la topologie PAC mémorisée et redémarre (nouvelle découverte)
 *   reboot                 : redémarrage
 */
static void run_serial_command(const String& cmd) {
    Serial.printf(">>> Commande serie : '%s'\r\n", cmd.c_str());

    if (cmd.equalsIgnoreCase("status") || cmd.equalsIgnoreCase("diag")) {
        log_bus_health();
        zb_bsb::ZbBridge::instance().dump_diagnostics();
    } else if (cmd.equalsIgnoreCase("report")) {
        zb_bsb::ZbBridge::instance().force_report_all();
        Serial.println("Rapports forces envoyes !");
    } else if (cmd.startsWith("get ")) {
        String field = cmd.substring(4);
        field.trim();
        uint32_t fid = 0;
        if (parse_field_id(field, fid)) {
            bsb::DecodedValue decoded; // Non utilisée : la réponse est affichée telle quelle
            bsb::BsbFrame     answer;
            if (g_bsb_bus.query_sync(fid, decoded, bsb::ValueType::UInt8, 3, 500, &answer)) {
                print_raw_answer(fid, answer);
            } else {
                Serial.printf("GET 0x%08lX -> pas de reponse (ou parametre refuse par le regulateur)\r\n",
                              static_cast<unsigned long>(fid));
            }
        } else {
            Serial.println("Usage : get <field_hex>   (ex: get 053D17DC)");
        }
    } else if (cmd.startsWith("set ")) {
        // Écriture brute sur la PAC : les deux arguments sont validés strictement, une faute de frappe
        // ne doit jamais se transformer en écriture de la valeur 0 ou en écriture sur le champ 0.
        String args = cmd.substring(4);
        args.trim();
        const int sp = args.indexOf(' ');
        String field = (sp > 0) ? args.substring(0, sp) : String();
        String value = (sp > 0) ? args.substring(sp + 1) : String();
        value.trim();
        uint32_t fid = 0;
        uint8_t  val = 0;
        if (parse_field_id(field, fid) && parse_uint8(value, val)) {
            const bool queued = g_bsb_bus.enqueue_set_uint8(fid, val);
            Serial.printf("SET 0x%08lX -> %u (%s)\r\n", static_cast<unsigned long>(fid), val,
                          queued ? "en file" : "file pleine");
        } else {
            Serial.println("Usage : set <field_hex> <0..255>   (ex: set 313D0571 1)");
        }
    } else if (cmd.startsWith("dhw ")) {
        const String arg = cmd.substring(4);
        long mode = -1;
        if (arg.equalsIgnoreCase("on")) {
            mode = static_cast<long>(bsb::DhwMode::On);
        } else if (arg.equalsIgnoreCase("off")) {
            mode = static_cast<long>(bsb::DhwMode::Off);
        } else if (arg.length() == 1 && isDigit(arg[0])) {
            mode = arg.toInt();
        }
        if (mode >= 0 && mode <= static_cast<long>(bsb::DhwMode::Eco)) {
            g_bsb_bus.enqueue_set_uint8(bsb::fields::DHW_MODE, static_cast<uint8_t>(mode));
            Serial.printf("Commande : BSB SET DHW_MODE -> %ld\r\n", mode);
        } else {
            Serial.println("Usage : dhw on|off|0|1|2");
        }
    } else if (cmd.equalsIgnoreCase("bus on")) {
        g_bsb_bus.resume();
        Serial.println("Bus BSB : emission et ecoute retablies.");
    } else if (cmd.equalsIgnoreCase("bus off") || cmd.startsWith("bus off ")) {
        // Simule un câble de bus débranché, sans toucher à l'installation : après 3 min les thermostats
        // doivent repasser à « inactif » et les mesures s'afficher indisponibles dans Home Assistant.
        String arg = cmd.substring(7);
        arg.trim();
        uint8_t minutes = 10;
        if (arg.length() > 0 && (!parse_uint8(arg, minutes) || minutes == 0 || minutes > 60)) {
            Serial.println("Usage : bus off [1..60 minutes] | bus on");
        } else {
            g_bsb_bus.pause(static_cast<uint32_t>(minutes) * 60000UL);
            Serial.printf("Bus BSB coupe pendant %u min (reprise automatique, ou 'bus on').\r\n", minutes);
        }
    } else if (cmd.equalsIgnoreCase("antenna int") || cmd.equalsIgnoreCase("antenna ext")) {
        // Permet de revenir sur l'autre antenne sans reset usine si le lien Zigbee a été perdu après une bascule
        const bool external = cmd.equalsIgnoreCase("antenna ext");
        zb_bsb::ZbBridge::instance().set_external_antenna_enabled(external);
        Serial.printf("Antenne RF -> %s\r\n", external ? "externe (U.FL)" : "interne (ceramique)");
    } else if (cmd.equalsIgnoreCase("rescan")) {
        // Même effet qu'un appui de 3 à 8 s sur le bouton BOOT
        Serial.println("Nouvelle decouverte de la PAC au redemarrage...");
        bsb::BsbDiscovery::request_rescan();
        delay(300);
        esp_restart();
    } else if (cmd.equalsIgnoreCase("reboot")) {
        Serial.println("Redemarrage...");
        delay(300);
        esp_restart();
    } else {
        Serial.println("Commandes : status | report | get <field_hex> | set <field_hex> <val> | dhw on|off|<mode> | bus off [min]|on | antenna int|ext | rescan | reboot");
    }
}

static void poll_serial_console() {
    static String s_cmd;
    static bool   s_overflow = false;

    while (Serial.available() > 0) {
        const int r = Serial.read();
        if (r < 0) break;
        const char c = static_cast<char>(r);
        if (c == '\r' || c == '\n') {
            s_cmd.trim();
            if (s_overflow) {
                Serial.println("Commande serie trop longue : ignoree.");
            } else if (s_cmd.length() > 0) {
                run_serial_command(s_cmd);
            }
            s_cmd      = "";
            s_overflow = false;
        } else if (s_cmd.length() >= SERIAL_CMD_MAX_LEN) {
            // Flux sans fin de ligne (bruit sur le port USB) : la mémoire consommée reste bornée
            s_overflow = true;
        } else if (isPrintable(c)) {
            s_cmd += c;
        }
    }
}

/**
 * @brief Garde la console USB utile quand plus personne ne la lit.
 *
 * Tant qu'un PC reste branché après la fermeture du moniteur série, le pilote USB considère la liaison
 * comme active : le tampon d'émission se remplit, les nouveaux messages sont perdus, et la réouverture
 * du moniteur n'afficherait que de vieux messages. Un tampon resté plein plusieurs secondes signifie
 * que personne ne lit : on le vide, ce qui fait repasser le pilote dans son mode « conserver les
 * messages les plus récents » jusqu'à la prochaine lecture.
 */
static void maintain_serial_console() {
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
    static uint32_t s_last_check_ms = 0;
    static uint8_t  s_full_checks   = 0;

    const uint32_t now = millis();
    if (now - s_last_check_ms < SERIAL_STALL_CHECK_MS) {
        return;
    }
    s_last_check_ms = now;

    if (Serial.availableForWrite() < static_cast<int>(SERIAL_TX_BUFFER_SIZE / 8)) {
        if (++s_full_checks >= 2) {
            s_full_checks = 0;
            Serial.flush(); // Avec un délai d'émission nul : vide le tampon et marque la console « non lue »
        }
    } else {
        s_full_checks = 0;
    }
#endif
}

void setup() {
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
    // La console USB ne doit JAMAIS ralentir la passerelle. Par défaut, dès qu'un PC est branché sans
    // qu'aucun programme ne lise le port (moniteur série refermé), chaque caractère de journal attend
    // jusqu'à 100 ms une place dans le tampon : la boucle BSB et la tâche Zigbee se figent, puis le
    // chien de garde redémarre l'appareil. Écriture non bloquante : ce qui ne tient pas est perdu.
    Serial.setTxBufferSize(SERIAL_TX_BUFFER_SIZE);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
#else
    Serial.begin(115200);
#endif
    // Redirige tous les logs ESP-IDF (ESP_LOGI, ESP_LOGW, etc.) vers la console USB CDC
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_INFO);

    // Attente brève (jusqu'à 2.5s) que le moniteur série USB du PC s'ouvre après le flash
    const uint32_t usb_wait_start = millis();
    while (!Serial && (millis() - usb_wait_start < 2500)) {
        delay(10);
    }
    delay(150);

    const esp_partition_t* running_part = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
    if (running_part && esp_ota_get_state_partition(running_part, &ota_state) == ESP_OK) {
        g_ota_pending_verify = (ota_state == ESP_OTA_IMG_PENDING_VERIFY);
    }

    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "  ZB-BSB — Passerelle Zigbee 3.0 pour Pompe a Chaleur BSB");
    ESP_LOGI(TAG, "  Materiel : Seeed Studio XIAO ESP32-C6 (D7=RX, D6=TX)");
    ESP_LOGI(TAG, "  Firmware OTA : 0x%08lX (Partition : %s%s)",
             static_cast<unsigned long>(ZB_BSB_OTA_FILE_VERSION),
             running_part ? running_part->label : "n/a",
             g_ota_pending_verify ? ", image en cours de validation" : "");
    ESP_LOGI(TAG, "==========================================================");

    // 1. Initialisation de la flash NVS, puis relevé de la cause de ce démarrage
    init_nvs();
    record_boot();
    if (is_abnormal_reset(g_reset_reason)) {
        ESP_LOGE(TAG, "  Demarrage n.%lu apres une PANNE : %s (%lu demarrage(s) anormal(aux) au total)",
                 static_cast<unsigned long>(g_boot_stats.boots), reset_reason_text(g_reset_reason),
                 static_cast<unsigned long>(g_boot_stats.abnormal));
    } else {
        ESP_LOGI(TAG, "  Demarrage n.%lu, cause : %s (%lu demarrage(s) anormal(aux) au total)",
                 static_cast<unsigned long>(g_boot_stats.boots), reset_reason_text(g_reset_reason),
                 static_cast<unsigned long>(g_boot_stats.abnormal));
    }

    // 2. Initialisation matérielle du XIAO ESP32-C6 (Commutateur RF persisté en NVS + GPIO)
    // Par défaut : OFF (false = Antenne céramique SMD intégrée)
    const bool use_ext_ant = zb_bsb::ZbBridge::load_external_antenna_nvs();
    zb_bsb::board::init_xiao_rf_antenna(use_ext_ant);
    zb_bsb::board::init_user_io();
    ESP_LOGI(TAG, "  Antenne RF : %s", use_ext_ant ? "ON (Externe U.FL)" : "OFF (Ceramique interne par defaut)");

    // 3. Initialisation du bus BSB matériel (UART1 : RX=GPIO17/D7, TX=GPIO16/D6, 4800 8O1)
    bsb::BsbBus::Config bus_cfg = {};
    bus_cfg.uart_port = zb_bsb::board::BSB_UART_PORT;
    bus_cfg.rx_pin    = zb_bsb::board::BSB_RX_PIN;
    bus_cfg.tx_pin    = zb_bsb::board::BSB_TX_PIN;
    bus_cfg.baud_rate = zb_bsb::board::BSB_BAUD_RATE;
    bus_cfg.src_addr  = zb_bsb::board::BSB_DEFAULT_SRC_ADDR;
    bus_cfg.dst_addr  = zb_bsb::board::BSB_DEFAULT_DST_ADDR;

    if (!g_bsb_bus.begin(bus_cfg)) {
        ESP_LOGE(TAG, "Erreur critique d'initialisation du bus BSB !");
    }

    // 4. Chargement de la topologie PAC depuis NVS ou exécution de l'Auto-Discovery
    bool topology_persisted = false;
    const bool rescan_requested = bsb::BsbDiscovery::rescan_requested();
    if (!rescan_requested && bsb::BsbDiscovery::load_from_nvs(g_caps)) {
        // La liste des organes vient de la NVS ; consignes, modes et températures sont relus tout de
        // suite pour que les attributs Zigbee ne démarrent pas sur un instantané périmé.
        bsb::BsbDiscovery::refresh_initial_values(g_bsb_bus, g_caps);
        ESP_LOGI(TAG, "Topologie PAC chargee depuis la memoire NVS :");
        bsb::BsbDiscovery::log_capabilities(g_caps);
        topology_persisted = true;
    } else {
        // Topologie déjà mémorisée (même d'un firmware antérieur) : elle sert de repli si la découverte
        // échoue, et de référence lors d'une migration automatique.
        bsb::BsbCapabilities previous;
        const bool have_previous = bsb::BsbDiscovery::load_from_nvs(previous, true);

        ESP_LOGI(TAG, "%s -> Lancement du scan Auto-Discovery sur le bus BSB...",
                 rescan_requested ? "Nouvelle decouverte demandee" : "Aucune topologie a jour en NVS");
        zb_bsb::board::set_status_led(true);
        if (discover_topology(g_caps)) {
            if (!rescan_requested && have_previous) {
                // Migration automatique après une mise à jour : elle ne doit jamais faire apparaître une
                // sonde d'ambiance filaire. Une valeur lisible là où il n'y avait rien est la température
                // injectée par la sonde externe, que le régulateur restitue comme une vraie mesure.
                bsb::ZoneCapabilities*       zones[]     = {&g_caps.zone1, &g_caps.zone2};
                const bsb::ZoneCapabilities* old_zones[] = {&previous.zone1, &previous.zone2};
                for (uint8_t i = 0; i < 2; ++i) {
                    if (old_zones[i]->present && !old_zones[i]->has_room_sensor && zones[i]->has_room_sensor) {
                        ESP_LOGW(TAG, "Zone %u : sonde d'ambiance vue par la decouverte = sonde externe injectee, ignoree.",
                                 static_cast<unsigned>(i + 1));
                        zones[i]->has_room_sensor = false;
                        zones[i]->initial_room_c  = zones[i]->initial_active_c;
                        bsb::BsbDiscovery::set_external_sensor_used(static_cast<uint8_t>(i + 1), true);
                    }
                }
            }
            topology_persisted = bsb::BsbDiscovery::save_to_nvs(g_caps);
            bsb::BsbDiscovery::clear_rescan_request();
        } else if (have_previous) {
            // Appareil déjà installé et PAC injoignable (ou scans discordants) à ce démarrage : on continue
            // avec la topologie précédente plutôt que de basculer sur un profil fictif. La découverte
            // reste due : elle sera retentée dès que le régulateur répondra, ou au prochain démarrage.
            g_caps = previous;
            ESP_LOGW(TAG, "Decouverte impossible : topologie precedente conservee pour ce demarrage :");
            bsb::BsbDiscovery::log_capabilities(g_caps);
            topology_persisted       = true;
            g_discovery_retry_wanted = true;
        } else {
            ESP_LOGW(TAG, "PAC non detectee sur le bus BSB -> Chargement du profil par defaut (non sauvegarde en NVS).");
            ESP_LOGW(TAG, "L'Auto-Discovery s'executera automatiquement des que la PAC repondra sur le bus.");
            load_bench_fallback_capabilities(g_caps);
            bsb::BsbDiscovery::log_capabilities(g_caps);
            g_discovery_retry_wanted = true;
        }
        zb_bsb::board::set_status_led(false);
    }

    // 5. Enregistrement des scrutations périodiques pour les seuls organes découverts
    bsb::BsbDiscovery::register_discovered_polls(g_bsb_bus, g_caps);

    // 6. Liaison temps-réel BSB -> Zigbee
    g_bsb_bus.set_value_callback([](const bsb::DecodedValue& val) {
        zb_bsb::ZbBridge::instance().on_bsb_value(val);
    });

    // 7. Vérification de la disposition Zigbee mémorisée. Le profil de secours (banc d'essai) est
    //    transitoire : il ne doit ni purger l'appairage existant ni devenir la référence.
    const bool need_zb_erase = topology_persisted && zigbee_layout_changed(g_caps);
    g_zigbee_nvram_purged   = need_zb_erase;
    g_layout_commit_pending = need_zb_erase;

    // 8. Démarrage de la pile Zigbee 3.0 avec création dynamique des Endpoints
    if (!zb_bsb::ZbBridge::instance().begin(g_caps, &g_bsb_bus, need_zb_erase)) {
        ESP_LOGE(TAG, "Echec du demarrage de la pile Zigbee !");
    }

    // 9. Chien de garde : si loop() cesse de tourner (interblocage, attente infinie), l'appareil redémarre
    esp_task_wdt_config_t wdt_cfg = {};
    wdt_cfg.timeout_ms     = LOOP_WATCHDOG_TIMEOUT_MS;
    wdt_cfg.idle_core_mask = 0;
    wdt_cfg.trigger_panic  = true;
    if (esp_task_wdt_reconfigure(&wdt_cfg) == ESP_OK) {
        enableLoopWDT();
    } else {
        ESP_LOGW(TAG, "Chien de garde de la boucle principale non active.");
    }
}

void loop() {
    // Gestion du bouton BOOT (Re-Discovery 3s / Factory Reset 8s)
    check_boot_button();

    // Gestion de l'état de la LED de statut (Allumée fixe si connecté Zigbee, clignotante 500ms si en recherche)
    update_status_led();

    // Exécution non-bloquante de l'automate BSB (lecture, snooping 0x0A, écritures SET, scrutation GET)
    g_bsb_bus.loop();

    // Maintenance périodique des sondes d'ambiance externes Home Assistant (ré-émission INF 60s & watchdog)
    zb_bsb::ZbBridge::instance().loop_housekeeping();

    // Validation (ou annulation) d'une image OTA fraîchement installée
    confirm_ota_image_when_stable();

    // Nouvelle disposition Zigbee : l'empreinte n'est enregistrée qu'une fois la pile démarrée, purge faite
    if (g_layout_commit_pending && zb_bsb::ZbBridge::instance().is_stack_started()) {
        g_layout_commit_pending = false;
        commit_zigbee_layout(g_caps);
    }

    // Découverte manquée au démarrage (PAC coupée) : dès que le régulateur répond à nos lectures, le bus
    // fonctionne dans les deux sens et un redémarrage la relance. Une seule tentative automatique par
    // mise sous tension : après ce redémarrage logiciel, un nouvel échec n'en provoque pas d'autre.
    if (g_discovery_retry_wanted && g_reset_reason != ESP_RST_SW && millis() > 30000UL &&
        g_bsb_bus.get_answered_count() >= 3 && !zb_bsb::ZbBridge::instance().is_ota_in_progress()) {
        ESP_LOGW(TAG, "Le regulateur repond de nouveau : redemarrage pour executer la decouverte de la PAC...");
        delay(300);
        esp_restart();
    }

    // Journal de diagnostic périodique (toutes les 15s au début puis 30s)
    const uint32_t now = millis();
    const uint32_t diag_interval = (now < 60000) ? 15000 : 30000;
    if (now - g_last_diag_ms >= diag_interval) {
        g_last_diag_ms = now;
        log_bus_health();
        zb_bsb::ZbBridge::instance().dump_diagnostics();
    }

    // Console série de diagnostic (USB)
    poll_serial_console();
    maintain_serial_console();

    delay(5);
}

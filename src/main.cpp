#include <Arduino.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "board_config.h"
#include "bsb_types.h"
#include "bsb_bus.h"
#include "bsb_discovery.h"
#include "zigbee/zb_bridge.h"

static const char* TAG = "zb_bsb.main";

static bsb::BsbBus          g_bsb_bus;
static bsb::BsbCapabilities g_caps;
static uint32_t             g_last_diag_ms = 0;
static uint32_t             g_btn_press_start_ms = 0;
static bool                 g_btn_pressed = false;

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

static void check_boot_button() {
    if (zb_bsb::ZbBridge::instance().is_ota_in_progress()) {
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
            bsb::BsbDiscovery::clear_nvs();
            esp_zb_factory_reset();
            delay(500);
            esp_restart();
        } else if (held_ms >= 3000) {
            ESP_LOGW(TAG, "Appui long (3-8s) : Relance de l'Auto-Decouverte PAC BSB...");
            zb_bsb::board::set_status_led(false);
            bsb::BsbDiscovery::clear_nvs();
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
    if (zb_bsb::ZbBridge::instance().is_ota_in_progress()) {
        // Mise à jour OTA en cours : clignotement soutenu (200 ms)
        zb_bsb::board::set_status_led(((now / 200) % 2) == 0);
    } else if (zb_bsb::ZbBridge::instance().is_connected()) {
        // Associé et connecté au réseau Zigbee : LED allumée fixe
        zb_bsb::board::set_status_led(true);
    } else {
        // En recherche / appairage Zigbee : clignotement régulier (500 ms)
        zb_bsb::board::set_status_led(((now / 500) % 2) == 0);
    }
}

void setup() {
    Serial.begin(115200);
    // Redirige tous les logs ESP-IDF (ESP_LOGI, ESP_LOGW, etc.) vers la console USB CDC
    Serial.setDebugOutput(true);

    // Attente brève (jusqu'à 2.5s) que le moniteur série USB du PC s'ouvre après le flash
    const uint32_t usb_wait_start = millis();
    while (!Serial && (millis() - usb_wait_start < 2500)) {
        delay(10);
    }
    delay(150);

    const esp_partition_t* running_part = esp_ota_get_running_partition();
    esp_ota_mark_app_valid_cancel_rollback();

    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "  ZB-BSB — Passerelle Zigbee 3.0 pour Pompe a Chaleur BSB");
    ESP_LOGI(TAG, "  Materiel : Seeed Studio XIAO ESP32-C6 (D7=RX, D6=TX)");
    ESP_LOGI(TAG, "  Firmware OTA : 0x%08lX (Partition : %s)",
             static_cast<unsigned long>(ZB_BSB_OTA_FILE_VERSION),
             running_part ? running_part->label : "n/a");
    ESP_LOGI(TAG, "==========================================================");

    // 1. Initialisation matérielle du XIAO ESP32-C6 (Commutateur RF + GPIO)
    zb_bsb::board::init_xiao_rf_antenna(false); // Antenne céramique interne par défaut
    zb_bsb::board::init_user_io();

    // 2. Initialisation de la flash NVS
    init_nvs();

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
    if (bsb::BsbDiscovery::load_from_nvs(g_caps)) {
        ESP_LOGI(TAG, "Topologie PAC chargee depuis la memoire NVS :");
        bsb::BsbDiscovery::log_capabilities(g_caps);
    } else {
        ESP_LOGI(TAG, "Aucune topologie en NVS -> Lancement du scan Auto-Discovery sur le bus BSB...");
        zb_bsb::board::set_status_led(true);
        if (bsb::BsbDiscovery::scan_heat_pump(g_bsb_bus, g_caps)) {
            bsb::BsbDiscovery::save_to_nvs(g_caps);
        } else {
            ESP_LOGW(TAG, "PAC non detectee sur le bus BSB -> Chargement du profil par defaut (non sauvegarde en NVS).");
            ESP_LOGW(TAG, "L'Auto-Discovery s'executera automatiquement des que le XIAO sera raccorde a la PAC.");
            load_bench_fallback_capabilities(g_caps);
            bsb::BsbDiscovery::log_capabilities(g_caps);
        }
        zb_bsb::board::set_status_led(false);
    }

    // 5. Enregistrement des scrutations périodiques pour les seuls organes découverts
    bsb::BsbDiscovery::register_discovered_polls(g_bsb_bus, g_caps);

    // 6. Liaison temps-réel BSB -> Zigbee
    g_bsb_bus.set_value_callback([](const bsb::DecodedValue& val) {
        zb_bsb::ZbBridge::instance().on_bsb_value(val);
    });

    // 7. Verification de version de topologie Zigbee en NVS
    // En cas de changement de disposition (ex: Thermostat ECS v4), un nettoyage NVRAM est effectue
    // pour purger les anciennes entrees de reporting stale qui causeraient un crash ZBOSS.
    bool need_zb_erase = false;
    nvs_handle_t nvs_h = 0;
    if (nvs_open("bsb_topo", NVS_READWRITE, &nvs_h) == ESP_OK) {
        uint16_t saved_ver = 0;
        if (nvs_get_u16(nvs_h, "zb_layout_ver", &saved_ver) != ESP_OK || saved_ver != 4) {
            ESP_LOGW(TAG, "Nouvelle disposition Zigbee (v4) : purge automatique de la NVRAM Zigbee stale...");
            need_zb_erase = true;
            nvs_set_u16(nvs_h, "zb_layout_ver", 4);
            nvs_commit(nvs_h);
        }
        nvs_close(nvs_h);
    }

    // 8. Démarrage de la pile Zigbee 3.0 avec création dynamique des Endpoints
    if (!zb_bsb::ZbBridge::instance().begin(g_caps, &g_bsb_bus, need_zb_erase)) {
        ESP_LOGE(TAG, "Echec du demarrage de la pile Zigbee !");
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

    // Journal de diagnostic périodique (toutes les 15s au début puis 30s)
    const uint32_t now = millis();
    const uint32_t diag_interval = (now < 60000) ? 15000 : 30000;
    if (now - g_last_diag_ms >= diag_interval) {
        g_last_diag_ms = now;
        ESP_LOGI(TAG, "[ Sante Bus BSB ] TX=%lu | RX=%lu | Collisions 0x0A evitees=%lu | Zigbee=%s",
                 static_cast<unsigned long>(g_bsb_bus.tx_count()),
                 static_cast<unsigned long>(g_bsb_bus.rx_count()),
                 static_cast<unsigned long>(g_bsb_bus.collision_avoided_count()),
                 zb_bsb::ZbBridge::instance().is_connected() ? "CONNECTE" : "EN RECHERCHE");
    }

    delay(5);
}

#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"

namespace zb_bsb {
namespace board {

// ============================================================================
// Brochage matériel — Seeed Studio XIAO ESP32-C6
// ============================================================================

// Interface UART BSB (4800 bauds, 8 bits, Parité IMPAIRE, 1 bit de stop)
// D7 = GPIO17 (RX), D6 = GPIO16 (TX)
static constexpr uart_port_t BSB_UART_PORT = UART_NUM_1;
static constexpr gpio_num_t  BSB_RX_PIN    = GPIO_NUM_17; // XIAO D7
static constexpr gpio_num_t  BSB_TX_PIN    = GPIO_NUM_16; // XIAO D6
static constexpr int         BSB_BAUD_RATE = 4800;

// Adresses sur le bus BSB
// 0x42 = Adresse maître secondaire pour notre passerelle ZB-BSB
//        (évite toute collision avec le bridge radio Cozytouch en 0x0A)
// 0x00 = Carte mère régulateur Siemens RVS21 / Alféa
static constexpr uint8_t BSB_DEFAULT_SRC_ADDR = 0x42;
static constexpr uint8_t BSB_DEFAULT_DST_ADDR = 0x00;

// ============================================================================
// Paramètres Zigbee OTA (Over-The-Air Firmware Upgrade — Cluster 0x0019)
// ============================================================================
#ifndef ZB_BSB_OTA_FILE_VERSION
#define ZB_BSB_OTA_FILE_VERSION      0x01010000UL // Version 1.1.0.0 (incrémenter pour publier une MAJ OTA)
#endif

#ifndef ZB_BSB_OTA_MANUFACTURER_CODE
#define ZB_BSB_OTA_MANUFACTURER_CODE 0x131B       // 4891 (Code constructeur Zigbee)
#endif

#ifndef ZB_BSB_OTA_IMAGE_TYPE
#define ZB_BSB_OTA_IMAGE_TYPE        0x0001       // 1 = Image ZB-BSB (Seeed Studio XIAO ESP32-C6)
#endif

#ifndef ZB_BSB_OTA_HW_VERSION
#define ZB_BSB_OTA_HW_VERSION        0x0001       // Révision matérielle 1
#endif

#ifndef ZB_BSB_OTA_MAX_DATA_SIZE
#define ZB_BSB_OTA_MAX_DATA_SIZE     223          // Taille max bloc OTA ZCL
#endif

#ifndef ZB_BSB_OTA_QUERY_INTERVAL_MIN
#define ZB_BSB_OTA_QUERY_INTERVAL_MIN 60          // Interrogation périodique auto (60 minutes)
#endif

// LED utilisateur intégrée au XIAO ESP32-C6 (Active à l'état BAS)
static constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_15;

// Bouton BOOT intégré au XIAO ESP32-C6 (Actif à l'état BAS)
// - Appui long (3s à 8s) : Relance une Auto-Découverte complète de la PAC
// - Appui très long (>8s) : Réinitialisation usine Zigbee + re-découverte
static constexpr gpio_num_t PIN_BOOT_BUTTON = GPIO_NUM_9;

// Commutateur d'antenne RF intégré au XIAO ESP32-C6 (puce FM8625H)
// GPIO3  = Activation du switch RF (doit être mis à LOW)
// GPIO14 = Sélection d'antenne (LOW = antenne céramique interne, HIGH = U.FL externe)
static constexpr gpio_num_t PIN_RF_SWITCH_EN  = GPIO_NUM_3;
static constexpr gpio_num_t PIN_RF_SWITCH_SEL = GPIO_NUM_14;

/**
 * @brief Bascule à chaud le commutateur d'antenne RF du Seeed Studio XIAO ESP32-C6.
 * @param use_external_antenna false = antenne céramique SMD intégrée (par défaut),
 *                             true  = connecteur U.FL externe.
 */
inline void set_xiao_rf_antenna(bool use_external_antenna) {
    // Activer la puce RF switch (GPIO3 = LOW)
    gpio_set_level(PIN_RF_SWITCH_EN, 0);
    // Sélectionner l'antenne (0 = céramique interne, 1 = U.FL externe)
    gpio_set_level(PIN_RF_SWITCH_SEL, use_external_antenna ? 1 : 0);
}

/**
 * @brief Configure le commutateur d'antenne RF du Seeed Studio XIAO ESP32-C6.
 * @param use_external_antenna false = antenne céramique SMD intégrée (par défaut),
 *                             true  = connecteur U.FL externe.
 */
inline void init_xiao_rf_antenna(bool use_external_antenna = false) {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_RF_SWITCH_EN) | (1ULL << PIN_RF_SWITCH_SEL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    set_xiao_rf_antenna(use_external_antenna);
}

/**
 * @brief Initialise la LED utilisateur et le bouton BOOT.
 */
inline void init_user_io() {
    gpio_config_t led_conf = {};
    led_conf.intr_type    = GPIO_INTR_DISABLE;
    led_conf.mode         = GPIO_MODE_OUTPUT;
    led_conf.pin_bit_mask = (1ULL << PIN_USER_LED);
    gpio_config(&led_conf);
    gpio_set_level(PIN_USER_LED, 1); // Éteinte par défaut (active low)

    gpio_config_t btn_conf = {};
    btn_conf.intr_type    = GPIO_INTR_DISABLE;
    btn_conf.mode         = GPIO_MODE_INPUT;
    btn_conf.pin_bit_mask = (1ULL << PIN_BOOT_BUTTON);
    btn_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&btn_conf);
}

inline void set_status_led(bool on) {
    gpio_set_level(PIN_USER_LED, on ? 0 : 1);
}

} // namespace board
} // namespace zb_bsb

#pragma once

#include <cstdint>
#include <cstddef>

namespace bsb {

// ============================================================================
// Types de commandes du protocole BSB (Boiler System Bus)
// ============================================================================
enum class Command : uint8_t {
    None = 0x00,
    Inf  = 0x02, // Diffusion d'information sans acquittement (Broadcast)
    Set  = 0x03, // Écriture d'un paramètre vers le régulateur
    Ack  = 0x04, // Acquittement positif d'un SET
    Nack = 0x05, // Acquittement négatif (erreur / non supporté)
    Get  = 0x06, // Lecture d'un paramètre
    Ret  = 0x07  // Réponse du régulateur à un GET
};

// ============================================================================
// Types de données BSB
// ============================================================================
enum class ValueType : uint8_t {
    Unknown = 0,
    UInt8,           // Entier 8 bits non signé / Enum (payload = [flag, val])
    Int8,            // Entier 8 bits signé (payload = [flag, val])
    Int16,           // Entier 16 bits signé big-endian (payload = [flag, msb, lsb])
    Int32,           // Entier 32 bits signé big-endian (payload = [flag, b3, b2, b1, b0])
    Temperature,     // Température BSB : int16_t / 64.0 °C (payload = [flag, msb, lsb])
    RoomTemperature, // Température d'ambiance diffusée via INF (payload = [msb, lsb, 0x00])
    Binary,          // État binaire (0x00 = OFF, 0xFF/non-zéro = ON)
    String           // Chaîne ASCII terminée par '\0'
};

// ============================================================================
// Dictionnaire officiel des Field IDs BSB (Siemens RVS21 / Atlantic Alféa)
// ============================================================================
namespace fields {

    // --- Identification Système ---
    static constexpr uint32_t SYSTEM_TYPE          = 0x053D0000; // Prog 6223 : Type système (20 = PAC)
    static constexpr uint32_t CONTROLLER_MODEL     = 0x053D0001; // Prog 6224 : Référence régulateur (ex: RVS21.831F/127)

    // --- Capteurs Globaux & Hydrauliques PAC ---
    static constexpr uint32_t OUTDOOR_TEMP         = 0x053D0521; // Prog 8700 : Température extérieure (°C)
    static constexpr uint32_t HP_FLOW_TEMP         = 0x593D052D; // Prog 8412 : Température départ PAC (°C)
    static constexpr uint32_t HP_RETURN_TEMP       = 0x593D0537; // Prog 8410 : Température retour PAC (°C)
    static constexpr uint32_t COMPRESSOR_MOD       = 0x053D0D2A; // Prog 8413 : Modulation du compresseur (%)
    static constexpr uint32_t BACKUP_MOD           = 0x053D12F4; // Prog 8414 : Modulation appoint électrique (%)
    static constexpr uint32_t COMPRESSOR_STATE     = 0x053D09B5; // Prog 8400 : État Compresseur 1 (0/255)
    static constexpr uint32_t HP_PUMP_STATE        = 0x053D09B9; // Prog 8406 : Pompe condenseur / circulateur principal PAC (Q9)
    static constexpr uint32_t BACKUP_HEATER_STATE  = 0x053D0AD1; // Prog 8402 : État Appoint électrique (0/255)
    static constexpr uint32_t HP_STATUS_CODE       = 0x053D17DC; // Prog 8006 : Code état Pompe à Chaleur

    // --- Zone 1 (Circuit Chauffage 1 — ex: Radiateurs) ---
    static constexpr uint32_t Z1_MODE              = 0x2D3D0574; // Prog 700  : Mode Zone 1 (0=Hors-gel, 1=Auto, 2=Réduit, 3=Confort)
    static constexpr uint32_t Z1_COMFORT_SETPOINT  = 0x2D3D058E; // Prog 710  : Consigne Confort Zone 1 (°C)
    static constexpr uint32_t Z1_REDUCED_SETPOINT  = 0x2D3D0590; // Prog 712  : Consigne Réduite Zone 1 (°C)
    static constexpr uint32_t Z1_ACTIVE_SETPOINT   = 0x2D3D0593; // Prog 8741 : Consigne active actuelle Zone 1 (°C)
    static constexpr uint32_t Z1_ROOM_TEMP         = 0x2D3D051E; // Prog 8740 : Température ambiante Zone 1 (°C)
    static constexpr uint32_t Z1_FLOW_TEMP         = 0x213D0518; // Prog 8743 : Température départ Zone 1 (°C)
    static constexpr uint32_t Z1_PUMP_STATE        = 0x053D09A5; // Prog 8730 : Pompe circuit chauffage Q2 Zone 1
    static constexpr uint32_t Z1_STATUS_CODE       = 0x053D07A3; // Prog 8000 : Code état Circuit Chauffage 1

    // --- Zone 2 (Circuit Chauffage 2 — ex: Plancher Chauffant sur AVS55) ---
    static constexpr uint32_t Z2_MODE              = 0x2E3D0574; // Prog 1000 : Mode Zone 2 (0=Hors-gel, 1=Auto, 2=Réduit, 3=Confort)
    static constexpr uint32_t Z2_COMFORT_SETPOINT  = 0x2E3D058E; // Prog 1010 : Consigne Confort Zone 2 (°C)
    static constexpr uint32_t Z2_REDUCED_SETPOINT  = 0x2E3D0590; // Prog 1012 : Consigne Réduite Zone 2 (°C)
    static constexpr uint32_t Z2_ACTIVE_SETPOINT   = 0x2E3D0593; // Prog 8771 : Consigne active actuelle Zone 2 (°C)
    static constexpr uint32_t Z2_ROOM_TEMP         = 0x2E3D051E; // Prog 8770 : Température ambiante Zone 2 (°C)
    static constexpr uint32_t Z2_FLOW_TEMP         = 0x223D0518; // Prog 8773 : Température départ Zone 2 (°C)
    static constexpr uint32_t Z2_PUMP_STATE        = 0x053D09A8; // Prog 8760 : Pompe circuit chauffage Q6 Zone 2
    static constexpr uint32_t Z2_STATUS_CODE       = 0x053D07A5; // Prog 8001 : Code état Circuit Chauffage 2

    // --- Eau Chaude Sanitaire (ECS / DHW) ---
    static constexpr uint32_t DHW_MODE             = 0x313D0574; // Prog 1600 : Mode ECS (0=Arrêt, 1=Marche, 2=Eco)
    static constexpr uint32_t DHW_NOMINAL_SETPOINT = 0x313D06B9; // Prog 1610 : Consigne nominale ECS (°C)
    static constexpr uint32_t DHW_REDUCED_SETPOINT = 0x313D06BA; // Prog 1612 : Consigne réduite ECS (°C)
    static constexpr uint32_t DHW_TANK_TEMP        = 0x313D052F; // Prog 8830 : Température ballon ECS B3 (°C)
    static constexpr uint32_t DHW_STATUS_CODE      = 0x053D07A1; // Prog 8003 : Code état ECS

} // namespace fields

// ============================================================================
// Modes de fonctionnement chauffage BSB (Programmes 700 / 1000)
// ============================================================================
enum class HeatingMode : uint8_t {
    Protection = 0, // Hors-gel / Protection
    Auto       = 1, // Automatique (horloge interne)
    Reduced    = 2, // Réduit / Éco permanent
    Comfort    = 3  // Confort permanent
};

// ============================================================================
// Modes de fonctionnement ECS (Programme 1600)
// ============================================================================
enum class DhwMode : uint8_t {
    Off = 0, // Arrêt
    On  = 1, // Marche
    Eco = 2  // Éco / Réduit
};

// ============================================================================
// Topologie matérielle découverte automatiquement sur le bus BSB (sauvée en NVS)
// ============================================================================
static constexpr uint32_t CAPABILITIES_MAGIC   = 0x5A424253; // "ZBBS"
static constexpr uint16_t CAPABILITIES_VERSION = 3;

struct ZoneCapabilities {
    bool  present           = false; // Le circuit répond aux lectures de mode/consigne
    bool  has_room_sensor   = false; // Sonde d'ambiance physique connectée (non 50.0°C / non désactivée)
    bool  has_flow_sensor   = false; // Sonde de départ dédiée connectée (non 0.0°C / non désactivée)
    bool  has_pump_state    = false; // État pompe du circuit disponible
    bool  has_status_code   = false; // Code statut du circuit disponible
    float initial_comfort_c = 20.0f;
    float initial_reduced_c = 18.0f;
    float initial_active_c  = 20.0f;
    float initial_room_c    = 20.0f;
    float initial_flow_c    = 25.0f;
    uint8_t initial_mode    = static_cast<uint8_t>(HeatingMode::Auto);
};

struct BsbCapabilities {
    uint32_t magic               = CAPABILITIES_MAGIC;
    uint16_t version             = CAPABILITIES_VERSION;
    char     controller_model[24] = "Siemens RVS";
    int16_t  system_type         = 0;

    // Capteurs & Organes Généraux PAC
    bool  has_outdoor_temp       = false;
    bool  has_hp_flow_temp       = false;
    bool  has_hp_return_temp     = false;
    bool  has_compressor_state   = false;
    bool  has_compressor_mod     = false;
    bool  has_hp_pump_state      = false; // Prog 8406 : Circulateur principal PAC (Q9)
    bool  has_backup_heater      = false;
    bool  has_backup_mod         = false;
    bool  has_hp_status_code     = false;

    float initial_outdoor_c      = 15.0f;
    float initial_hp_flow_c      = 25.0f;
    float initial_hp_return_c    = 25.0f;

    // Zones de chauffage
    ZoneCapabilities zone1;
    ZoneCapabilities zone2;

    // Eau Chaude Sanitaire (Ballon ECS)
    bool  has_dhw                  = false;
    bool  has_dhw_tank_temp        = false;
    bool  has_dhw_status_code      = false;
    bool  has_dhw_reduced_setpoint = false;
    float initial_dhw_setpoint_c   = 50.0f;
    float initial_dhw_reduced_c    = 40.0f;
    float initial_dhw_tank_c       = 45.0f;
    uint8_t initial_dhw_mode       = static_cast<uint8_t>(DhwMode::On);
};

// ============================================================================
// Valeur décodée d'un paquet BSB
// ============================================================================
struct DecodedValue {
    uint32_t  field_id   = 0;
    ValueType type       = ValueType::Unknown;
    bool      valid      = false; // true si le paquet est valide et que la sonde n'est pas désactivée
    uint8_t   flag_byte  = 0;     // 0x00 = normal, != 0 = sonde absente/désactivée en lecture RET
    float     float_val  = 0.0f;
    int32_t   int_val    = 0;
    bool      bool_val   = false;
    char      str_val[32] = {0};
};

// ============================================================================
// Fonctions utilitaires de traduction, validation sentinelle et métadonnées
// ============================================================================
ValueType   get_field_value_type(uint32_t field_id);
const char* get_field_name(uint32_t field_id);
const char* get_hp_status_text_fr(uint16_t code);
const char* get_zone_status_text_fr(uint16_t code);
const char* get_dhw_status_text_fr(uint16_t code);

/**
 * @brief Vérifie si une mesure de sonde d'ambiance est physiquement valide
 *        (filtre les sondes absentes renvoyant flag_byte != 0 ou la sentinelle 50.0°C).
 */
bool is_valid_room_sensor(const DecodedValue& v);

/**
 * @brief Vérifie si une mesure de sonde de départ hydraulique est physiquement valide
 *        (filtre les sondes absentes renvoyant flag_byte != 0 ou la sentinelle 0.0°C).
 */
bool is_valid_flow_sensor(const DecodedValue& v);

/**
 * @brief Vérifie si une consigne de température se situe dans une plage réaliste.
 */
bool is_valid_setpoint(const DecodedValue& v, float min_c, float max_c);

} // namespace bsb

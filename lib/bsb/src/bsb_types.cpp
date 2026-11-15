#include "bsb_types.h"

namespace bsb {

ValueType get_field_value_type(uint32_t field_id) {
    using namespace fields;
    switch (field_id) {
        case CONTROLLER_MODEL:
            return ValueType::String;

        case SYSTEM_TYPE:
            return ValueType::Int16;

        case OUTDOOR_TEMP:
        case HP_FLOW_TEMP:
        case HP_RETURN_TEMP:
        case Z1_COMFORT_SETPOINT:
        case Z1_REDUCED_SETPOINT:
        case Z1_ACTIVE_SETPOINT:
        case Z1_ROOM_TEMP:
        case Z1_FLOW_TEMP:
        case Z2_COMFORT_SETPOINT:
        case Z2_REDUCED_SETPOINT:
        case Z2_ACTIVE_SETPOINT:
        case Z2_ROOM_TEMP:
        case Z2_FLOW_TEMP:
        case DHW_NOMINAL_SETPOINT:
        case DHW_REDUCED_SETPOINT:
        case DHW_TANK_TEMP:
            return ValueType::Temperature;

        case COMPRESSOR_STATE:
        case HP_PUMP_STATE:
        case BACKUP_HEATER_STATE:
        case Z1_PUMP_STATE:
        case Z2_PUMP_STATE:
            return ValueType::Binary;

        case COMPRESSOR_MOD:
        case BACKUP_MOD:
        case Z1_MODE:
        case Z2_MODE:
        case DHW_MODE:
        case Z1_STATUS_CODE:
        case Z2_STATUS_CODE:
        case DHW_STATUS_CODE:
        case HP_STATUS_CODE:
            return ValueType::UInt8;

        default:
            return ValueType::Unknown;
    }
}

const char* get_field_name(uint32_t field_id) {
    using namespace fields;
    switch (field_id) {
        case SYSTEM_TYPE:          return "Type Systeme (6223)";
        case CONTROLLER_MODEL:     return "Modele Regulateur (6224)";
        case OUTDOOR_TEMP:         return "Temp. Exterieure (8700)";
        case HP_FLOW_TEMP:         return "Temp. Depart PAC (8412)";
        case HP_RETURN_TEMP:       return "Temp. Retour PAC (8410)";
        case COMPRESSOR_MOD:       return "Modulation Compresseur (8413)";
        case BACKUP_MOD:           return "Modulation Appoint (8414)";
        case COMPRESSOR_STATE:     return "Etat Compresseur 1 (8400)";
        case HP_PUMP_STATE:        return "Circulateur Principal PAC (8406)";
        case BACKUP_HEATER_STATE:  return "Etat Appoint Electrique (8402)";
        case HP_STATUS_CODE:       return "Statut PAC (8006)";
        case Z1_MODE:              return "Mode Zone 1 (700)";
        case Z1_COMFORT_SETPOINT:  return "Consigne Confort Zone 1 (710)";
        case Z1_REDUCED_SETPOINT:  return "Consigne Reduite Zone 1 (712)";
        case Z1_ACTIVE_SETPOINT:   return "Consigne Active Zone 1 (8741)";
        case Z1_ROOM_TEMP:         return "Temp. Ambiante Zone 1 (8740)";
        case Z1_FLOW_TEMP:         return "Temp. Depart Zone 1 (8743)";
        case Z1_PUMP_STATE:        return "Pompe Zone 1 Q2 (8730)";
        case Z1_STATUS_CODE:       return "Statut Zone 1 (8000)";
        case Z1_ROOM_TEMP_INF:     return "Injection Temp. Ambiante Zone 1 (10000)";
        case Z2_MODE:              return "Mode Zone 2 (1000)";
        case Z2_COMFORT_SETPOINT:  return "Consigne Confort Zone 2 (1010)";
        case Z2_REDUCED_SETPOINT:  return "Consigne Reduite Zone 2 (1012)";
        case Z2_ACTIVE_SETPOINT:   return "Consigne Active Zone 2 (8771)";
        case Z2_ROOM_TEMP:         return "Temp. Ambiante Zone 2 (8770)";
        case Z2_FLOW_TEMP:         return "Temp. Depart Zone 2 (8773)";
        case Z2_PUMP_STATE:        return "Pompe Zone 2 Q6 (8760)";
        case Z2_STATUS_CODE:       return "Statut Zone 2 (8001)";
        case Z2_ROOM_TEMP_INF:     return "Injection Temp. Ambiante Zone 2 (10001)";
        case DHW_MODE:             return "Mode ECS (1600)";
        case DHW_NOMINAL_SETPOINT: return "Consigne Nominale ECS (1610)";
        case DHW_REDUCED_SETPOINT: return "Consigne Reduite ECS (1612)";
        case DHW_TANK_TEMP:        return "Temp. Ballon ECS (8830)";
        case DHW_STATUS_CODE:      return "Statut ECS (8003)";
        default:                   return "Champ BSB Inconnu";
    }
}

bool is_valid_room_sensor(const DecodedValue& v) {
    // Sur régulateurs Siemens RVS21, une sonde d'ambiance non câblée renvoie :
    // - soit flag_byte != 0x00 ("---")
    // - soit la valeur sentinelle fixe 50.0 °C (raw = 3200)
    return v.valid && (v.flag_byte == 0x00) && (v.float_val >= -5.0f && v.float_val <= 44.0f);
}

bool is_valid_flow_sensor(const DecodedValue& v) {
    // Une sonde de départ de zone non câblée (circuit direct sans vanne mélangeuse) renvoie :
    // - soit flag_byte != 0x00 ("---")
    // - soit 0.0 °C
    return v.valid && (v.flag_byte == 0x00) && (v.float_val >= 4.0f && v.float_val <= 95.0f);
}

bool is_valid_setpoint(const DecodedValue& v, float min_c, float max_c) {
    return v.valid && (v.float_val >= min_c && v.float_val <= max_c);
}

} // namespace bsb

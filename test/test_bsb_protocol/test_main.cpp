#include <unity.h>
#include <cstring>
#include <cmath>
#include "bsb_types.h"
#include "bsb_frame.h"

using namespace bsb;

void setUp(void) {}
void tearDown(void) {}

// ============================================================================
// 1. Tests du calcul CRC-16 CCITT (XMODEM)
// ============================================================================
void test_crc16_xmodem_known_vectors(void) {
    // Vecteur standard ASCII "123456789" -> CRC-16 XMODEM = 0x31C3
    const uint8_t ascii_vec[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x31C3, BsbFrame::compute_crc16(ascii_vec, sizeof(ascii_vec)));

    // Trame BSB réelle capturée sur RVS21.831F/127 : GET 0x053D0521 (0x42 -> 0x00)
    // Octets : DC C2 00 0B 06 3D 05 05 21, CRC = 0xB97C
    const uint8_t bsb_get_hdr[] = {0xDC, 0xC2, 0x00, 0x0B, 0x06, 0x3D, 0x05, 0x05, 0x21};
    TEST_ASSERT_EQUAL_HEX16(0xB97C, BsbFrame::compute_crc16(bsb_get_hdr, sizeof(bsb_get_hdr)));
}

// ============================================================================
// 2. Tests de sérialisation GET et permutation des 2 premiers octets du field_id
// ============================================================================
void test_frame_make_get_swaps_msb_bytes(void) {
    BsbFrame f = BsbFrame::make_get(0x42, 0x00, fields::OUTDOOR_TEMP); // 0x053D0521

    TEST_ASSERT_EQUAL_UINT8(11, f.raw_len);
    TEST_ASSERT_EQUAL_HEX8(0xDC, f.raw[0]);
    TEST_ASSERT_EQUAL_HEX8(0xC2, f.raw[1]); // 0x42 | 0x80
    TEST_ASSERT_EQUAL_HEX8(0x00, f.raw[2]);
    TEST_ASSERT_EQUAL_HEX8(0x0B, f.raw[3]); // longueur = 11
    TEST_ASSERT_EQUAL_HEX8(0x06, f.raw[4]); // Command::Get
    // Permutation des 2 premiers octets de 0x053D0521 -> 3D 05 05 21
    TEST_ASSERT_EQUAL_HEX8(0x3D, f.raw[5]);
    TEST_ASSERT_EQUAL_HEX8(0x05, f.raw[6]);
    TEST_ASSERT_EQUAL_HEX8(0x05, f.raw[7]);
    TEST_ASSERT_EQUAL_HEX8(0x21, f.raw[8]);
    // CRC-16 XMODEM = 0xB97C
    TEST_ASSERT_EQUAL_HEX8(0xB9, f.raw[9]);
    TEST_ASSERT_EQUAL_HEX8(0x7C, f.raw[10]);
}

// ============================================================================
// 3. Tests de sérialisation SET Température (encodage * 64) et SET UInt8
// ============================================================================
void test_frame_make_set_temperature(void) {
    // Consigne Confort Zone 2 (0x2E3D058E) à 20.5 °C -> 20.5 * 64 = 1312 (0x0520)
    BsbFrame f = BsbFrame::make_set_temperature(0x42, 0x00, fields::Z2_COMFORT_SETPOINT, 20.5f);

    TEST_ASSERT_EQUAL_UINT8(14, f.raw_len);
    TEST_ASSERT_EQUAL_HEX8(0x03, f.raw[4]); // Command::Set
    // 0x2E3D058E permuté -> 3D 2E 05 8E
    TEST_ASSERT_EQUAL_HEX8(0x3D, f.raw[5]);
    TEST_ASSERT_EQUAL_HEX8(0x2E, f.raw[6]);
    TEST_ASSERT_EQUAL_HEX8(0x05, f.raw[7]);
    TEST_ASSERT_EQUAL_HEX8(0x8E, f.raw[8]);
    // Payload : [0x01 (enable), 0x05 (MSB), 0x20 (LSB)]
    TEST_ASSERT_EQUAL_HEX8(0x01, f.raw[9]);
    TEST_ASSERT_EQUAL_HEX8(0x05, f.raw[10]);
    TEST_ASSERT_EQUAL_HEX8(0x20, f.raw[11]);
}

void test_frame_make_set_uint8_mode(void) {
    // Mode Zone 1 (0x2D3D0574) -> Confort (3)
    BsbFrame f = BsbFrame::make_set_uint8(0x42, 0x00, fields::Z1_MODE, 3, 0x01);
    TEST_ASSERT_EQUAL_UINT8(13, f.raw_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, f.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, f.payload[1]);

    // Cas particulier enable_byte = 0x06 avec valeur 0 -> doit devenir 0x05
    BsbFrame f_off = BsbFrame::make_set_uint8(0x42, 0x00, fields::Z1_MODE, 0, 0x06);
    TEST_ASSERT_EQUAL_HEX8(0x05, f_off.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, f_off.payload[1]);
}

void test_frame_make_inf_room_temperature(void) {
    // Injection température ambiante Zone 1 (20.0 °C -> 1280 = 0x0500) en broadcast 0x7F
    BsbFrame f = BsbFrame::make_inf_room_temperature(0x42, fields::Z1_ROOM_TEMP, 20.0f);
    TEST_ASSERT_EQUAL_HEX8(0x7F, f.dst_addr);
    TEST_ASSERT_EQUAL_HEX8(0x02, static_cast<uint8_t>(f.command)); // Command::Inf
    TEST_ASSERT_EQUAL_UINT8(3, f.payload_len);
    TEST_ASSERT_EQUAL_HEX8(0x05, f.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, f.payload[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, f.payload[2]);
}

// ============================================================================
// 4. Tests de l'automate BsbFrameParser & décodage des températures (+/-)
// ============================================================================
static BsbFrame build_ret_frame(uint32_t field_id, const uint8_t* pl, uint8_t pl_len) {
    BsbFrame f;
    f.src_addr    = 0x00;
    f.dst_addr    = 0x42;
    f.command     = Command::Ret;
    f.field_id    = field_id;
    f.payload_len = pl_len;
    memcpy(f.payload, pl, pl_len);
    f.serialize();
    return f;
}

void test_parser_valid_ret_temperature_positive_and_negative(void) {
    BsbFrameParser parser;
    BsbFrame parsed;

    // 1) Température positive : +19.75 °C -> 19.75 * 64 = 1264 (0x04F0)
    const uint8_t pl_pos[] = {0x00, 0x04, 0xF0};
    BsbFrame tx_pos = build_ret_frame(fields::Z2_ROOM_TEMP, pl_pos, sizeof(pl_pos));

    bool got_frame = false;
    for (uint8_t i = 0; i < tx_pos.raw_len; ++i) {
        got_frame = parser.feed(tx_pos.raw[i], parsed);
    }
    TEST_ASSERT_TRUE(got_frame);
    TEST_ASSERT_EQUAL_HEX32(fields::Z2_ROOM_TEMP, parsed.field_id);

    DecodedValue val_pos = parsed.decode();
    TEST_ASSERT_TRUE(val_pos.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 19.75f, val_pos.float_val);

    // 2) Température négative : -5.50 °C -> -5.50 * 64 = -352 (0xFEA0 en complément à 2)
    const uint8_t pl_neg[] = {0x00, 0xFE, 0xA0};
    BsbFrame tx_neg = build_ret_frame(fields::OUTDOOR_TEMP, pl_neg, sizeof(pl_neg));

    got_frame = false;
    for (uint8_t i = 0; i < tx_neg.raw_len; ++i) {
        got_frame = parser.feed(tx_neg.raw[i], parsed);
    }
    TEST_ASSERT_TRUE(got_frame);
    DecodedValue val_neg = parsed.decode();
    TEST_ASSERT_TRUE(val_neg.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -5.50f, val_neg.float_val);
}

// ============================================================================
// 5. Test du Snooping passif Cozytouch (0x0A) et remise en ordre du field_id
// ============================================================================
void test_parser_cozytouch_snooping_get_unswaps_field_id(void) {
    BsbFrame cozy_get = BsbFrame::make_get(0x0A, 0x00, fields::DHW_TANK_TEMP); // 0x313D052F
    // Sur le fil, les octets 5 et 6 sont 3D 31
    TEST_ASSERT_EQUAL_HEX8(0x3D, cozy_get.raw[5]);
    TEST_ASSERT_EQUAL_HEX8(0x31, cozy_get.raw[6]);

    BsbFrameParser parser;
    BsbFrame rx;
    bool ok = false;
    for (uint8_t i = 0; i < cozy_get.raw_len; ++i) {
        ok = parser.feed(cozy_get.raw[i], rx);
    }
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(0x0A, rx.src_addr);
    // Le parseur doit avoir restauré l'identifiant canonique 0x313D052F
    TEST_ASSERT_EQUAL_HEX32(fields::DHW_TANK_TEMP, rx.field_id);
}

// ============================================================================
// 6. Tests de robustesse : CRC erroné, bruit sur le bus et longueurs invalides
// ============================================================================
void test_parser_rejects_bad_crc_and_recovers(void) {
    BsbFrameParser parser;
    BsbFrame parsed;

    const uint8_t pl[] = {0x00, 0x0B, 0x00}; // 44.0 °C
    BsbFrame bad_frame = build_ret_frame(fields::DHW_TANK_TEMP, pl, sizeof(pl));
    bad_frame.raw[bad_frame.raw_len - 1] ^= 0x55; // Corrompre le dernier octet du CRC

    bool ok = false;
    for (uint8_t i = 0; i < bad_frame.raw_len; ++i) {
        if (parser.feed(bad_frame.raw[i], parsed)) {
            ok = true;
        }
    }
    TEST_ASSERT_FALSE(ok);

    // La trame valide suivante doit être immédiatement acceptée sans perte de synchro
    BsbFrame good_frame = build_ret_frame(fields::DHW_TANK_TEMP, pl, sizeof(pl));
    for (uint8_t i = 0; i < good_frame.raw_len; ++i) {
        ok = parser.feed(good_frame.raw[i], parsed);
    }
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 44.0f, parsed.decode().float_val);
}

void test_parser_resynchronizes_on_bus_noise_and_invalid_length(void) {
    BsbFrameParser parser;
    BsbFrame parsed;

    // Bruit : faux 0xDC suivi d'une longueur trop courte (0x04), puis faux 0xDC suivi d'un second 0xDC
    const uint8_t noise[] = {0x00, 0xFF, 0xDC, 0x80, 0x42, 0x04, 0x12, 0xDC};
    for (size_t i = 0; i < sizeof(noise); ++i) {
        TEST_ASSERT_FALSE(parser.feed(noise[i], parsed));
    }

    // Suivi immédiatement d'une vraie trame GET valide
    BsbFrame valid_get = BsbFrame::make_get(0x42, 0x00, fields::OUTDOOR_TEMP);
    bool ok = false;
    for (uint8_t i = 0; i < valid_get.raw_len; ++i) {
        ok = parser.feed(valid_get.raw[i], parsed);
    }
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_HEX32(fields::OUTDOOR_TEMP, parsed.field_id);
}

// ============================================================================
// 7. Tests du décodage Chaîne (6224), Binaire (8400/8730) et Modulation (8413)
// ============================================================================
void test_decode_controller_model_string_strips_null(void) {
    const uint8_t str_pl[] = {'R', 'V', 'S', '2', '1', '.', '8', '3', '1', 'F', '/', '1', '2', '7', 0x00};
    BsbFrame f = build_ret_frame(fields::CONTROLLER_MODEL, str_pl, sizeof(str_pl));

    DecodedValue v = f.decode();
    TEST_ASSERT_TRUE(v.valid);
    TEST_ASSERT_EQUAL_STRING("RVS21.831F/127", v.str_val);
    TEST_ASSERT_EQUAL_UINT32(14, strlen(v.str_val));
}

void test_decode_binary_states_and_modulations(void) {
    // Pompe ON : payload = 00 FF
    const uint8_t pl_on[] = {0x00, 0xFF};
    DecodedValue v_on = build_ret_frame(fields::Z2_PUMP_STATE, pl_on, 2).decode();
    TEST_ASSERT_TRUE(v_on.valid);
    TEST_ASSERT_TRUE(v_on.bool_val);

    // Pompe OFF sur RVS21 : payload = 01 00
    const uint8_t pl_off[] = {0x01, 0x00};
    DecodedValue v_off = build_ret_frame(fields::Z1_PUMP_STATE, pl_off, 2).decode();
    TEST_ASSERT_TRUE(v_off.valid);
    TEST_ASSERT_FALSE(v_off.bool_val);

    // Pompe condenseur / circulateur principal PAC ON (8406) : payload = 00 FF
    DecodedValue v_hp_pump_on = build_ret_frame(fields::HP_PUMP_STATE, pl_on, 2).decode();
    TEST_ASSERT_TRUE(v_hp_pump_on.valid);
    TEST_ASSERT_TRUE(v_hp_pump_on.bool_val);

    // Modulation compresseur active (58%) : payload = 00 3A
    const uint8_t pl_mod_active[] = {0x00, 0x3A};
    DecodedValue v_mod = build_ret_frame(fields::COMPRESSOR_MOD, pl_mod_active, 2).decode();
    TEST_ASSERT_TRUE(v_mod.valid);
    TEST_ASSERT_EQUAL_INT32(58, v_mod.int_val);

    // Modulation compresseur à l'arrêt ("---" -> flag_byte = 0x01) : doit valoir 0%
    const uint8_t pl_mod_idle[] = {0x01, 0x00};
    DecodedValue v_mod_idle = build_ret_frame(fields::COMPRESSOR_MOD, pl_mod_idle, 2).decode();
    TEST_ASSERT_TRUE(v_mod_idle.valid);
    TEST_ASSERT_EQUAL_INT32(0, v_mod_idle.int_val);
}

// ============================================================================
// 8. Tests du filtrage des valeurs sentinelles pour l'Auto-Discovery
// ============================================================================
void test_autodiscovery_sentinel_filtering(void) {
    // Sonde d'ambiance Zone 1 absente : renvoie 50.0 °C (3200 = 0x0C80)
    const uint8_t pl_50c[] = {0x00, 0x0C, 0x80};
    DecodedValue v_z1_room = build_ret_frame(fields::Z1_ROOM_TEMP, pl_50c, 3).decode();
    TEST_ASSERT_TRUE(v_z1_room.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, v_z1_room.float_val);
    // L'Auto-Discovery DOIT rejeter cette sonde sentinelle 50.0°C !
    TEST_ASSERT_FALSE(is_valid_room_sensor(v_z1_room));

    // Sonde d'ambiance Zone 2 présente : renvoie 19.8 °C (1267 = 0x04F3)
    const uint8_t pl_19_8c[] = {0x00, 0x04, 0xF3};
    DecodedValue v_z2_room = build_ret_frame(fields::Z2_ROOM_TEMP, pl_19_8c, 3).decode();
    TEST_ASSERT_TRUE(is_valid_room_sensor(v_z2_room));

    // Sonde de départ Zone 1 absente : renvoie 0.0 °C (0x0000)
    const uint8_t pl_0c[] = {0x00, 0x00, 0x00};
    DecodedValue v_z1_flow = build_ret_frame(fields::Z1_FLOW_TEMP, pl_0c, 3).decode();
    TEST_ASSERT_FALSE(is_valid_flow_sensor(v_z1_flow));

    // Sonde de départ Zone 2 présente : renvoie 21.5 °C (1376 = 0x0560)
    const uint8_t pl_21_5c[] = {0x00, 0x05, 0x60};
    DecodedValue v_z2_flow = build_ret_frame(fields::Z2_FLOW_TEMP, pl_21_5c, 3).decode();
    TEST_ASSERT_TRUE(is_valid_flow_sensor(v_z2_flow));
}

// ============================================================================
// 9. Tests des tables de traduction des codes statut en français
// ============================================================================
void test_french_status_translations(void) {
    TEST_ASSERT_EQUAL_STRING("En fonctionnement (Chauffage/ECS)", get_hp_status_text_fr(44));
    TEST_ASSERT_EQUAL_STRING("Arret / Pas de demande de chaleur", get_hp_status_text_fr(51));
    TEST_ASSERT_EQUAL_STRING("Regime de chauffe Confort", get_zone_status_text_fr(101));
    TEST_ASSERT_EQUAL_STRING("Charge (Pret - Temp. atteinte)", get_dhw_status_text_fr(99));
}

static int run_all_bsb_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_xmodem_known_vectors);
    RUN_TEST(test_frame_make_get_swaps_msb_bytes);
    RUN_TEST(test_frame_make_set_temperature);
    RUN_TEST(test_frame_make_set_uint8_mode);
    RUN_TEST(test_frame_make_inf_room_temperature);
    RUN_TEST(test_parser_valid_ret_temperature_positive_and_negative);
    RUN_TEST(test_parser_cozytouch_snooping_get_unswaps_field_id);
    RUN_TEST(test_parser_rejects_bad_crc_and_recovers);
    RUN_TEST(test_parser_resynchronizes_on_bus_noise_and_invalid_length);
    RUN_TEST(test_decode_controller_model_string_strips_null);
    RUN_TEST(test_decode_binary_states_and_modulations);
    RUN_TEST(test_autodiscovery_sentinel_filtering);
    RUN_TEST(test_french_status_translations);
    return UNITY_END();
}

#if defined(ARDUINO)
#include <Arduino.h>
void setup() {
    delay(1000);
    run_all_bsb_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    return run_all_bsb_tests();
}
#endif

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

// Passe un flux d'octets dans l'automate et compte les trames rendues (la dernière reste dans `last`)
static int feed_stream(BsbFrameParser& parser, const uint8_t* bytes, size_t len, BsbFrame& last) {
    int frames = 0;
    for (size_t i = 0; i < len; ++i) {
        BsbFrame parsed;
        if (parser.feed(bytes[i], parsed)) {
            last = parsed;
            ++frames;
        }
    }
    return frames;
}

void test_parser_finds_a_frame_swallowed_whole_by_a_corrupted_length(void) {
    // Longueur corrompue de 14 en 25 : la trame absorbe les 11 octets de la suivante avant d'être rejetée.
    // La relecture de ses octets retrouve la trame suivante, entière.
    const uint8_t pl[]    = {0x00, 0x0B, 0x00}; // 44.0 °C
    BsbFrame      damaged = build_ret_frame(fields::DHW_TANK_TEMP, pl, sizeof(pl));
    const BsbFrame get    = BsbFrame::make_get(0x0A, 0x00, fields::HP_FLOW_TEMP);
    damaged.raw[3] = static_cast<uint8_t>(damaged.raw_len + get.raw_len);

    uint8_t stream[2 * BsbFrame::MAX_FRAME_LEN];
    memcpy(stream, damaged.raw, damaged.raw_len);
    memcpy(stream + damaged.raw_len, get.raw, get.raw_len);

    BsbFrameParser parser;
    BsbFrame       last;
    TEST_ASSERT_EQUAL_INT(1, feed_stream(parser, stream, static_cast<size_t>(damaged.raw_len) + get.raw_len, last));
    TEST_ASSERT_EQUAL_HEX8(0x0A, last.src_addr);
    TEST_ASSERT_EQUAL_HEX32(fields::HP_FLOW_TEMP, last.field_id);
    TEST_ASSERT_EQUAL_UINT32(1, parser.crc_error_count());
    TEST_ASSERT_EQUAL_UINT32(1, parser.recovered_count());
    TEST_ASSERT_FALSE(parser.in_frame());
}

void test_parser_is_not_fooled_by_a_data_byte_that_looks_like_a_frame_start(void) {
    // La trame amputée contient 0xDC dans ses données, suivi d'octets qui forment un en-tête plausible
    // (source 0x80, destination 0x00, longueur 11). La relecture part d'abord de ce faux début, qui absorbe
    // le commencement de la vraie trame suivante avant d'échouer : celle-ci doit être retrouvée quand même,
    // et cet essai manqué ne compte pas comme une seconde erreur du bus.
    const uint8_t pl[]     = {0x00, 0xDC, 0x80, 0x00, 0x0B, 0x01};
    const BsbFrame damaged = build_ret_frame(fields::CONTROLLER_MODEL, pl, sizeof(pl));
    const BsbFrame get     = BsbFrame::make_get(0x42, 0x00, fields::HP_RETURN_TEMP);

    uint8_t stream[2 * BsbFrame::MAX_FRAME_LEN];
    size_t  len = damaged.raw_len - 1U; // Dernier octet perdu
    memcpy(stream, damaged.raw, len);
    memcpy(stream + len, get.raw, get.raw_len);
    len += get.raw_len;

    BsbFrameParser parser;
    BsbFrame       last;
    TEST_ASSERT_EQUAL_INT(1, feed_stream(parser, stream, len, last));
    TEST_ASSERT_EQUAL_HEX8(0x42, last.src_addr);
    TEST_ASSERT_EQUAL_HEX32(fields::HP_RETURN_TEMP, last.field_id);
    TEST_ASSERT_EQUAL_UINT32(1, parser.crc_error_count());

    // Et le flux reste sain ensuite
    const BsbFrame next = BsbFrame::make_get(0x42, 0x00, fields::OUTDOOR_TEMP);
    TEST_ASSERT_EQUAL_INT(1, feed_stream(parser, next.raw, next.raw_len, last));
    TEST_ASSERT_EQUAL_HEX32(fields::OUTDOOR_TEMP, last.field_id);
    TEST_ASSERT_EQUAL_UINT32(1, parser.crc_error_count());
}

void test_parser_finds_a_frame_starting_inside_a_rejected_header(void) {
    // Deux octets parasites « DC 8A » juste avant une vraie trame : son 0xDC est pris pour une adresse de
    // destination, son octet suivant pour une longueur impossible. L'en-tête est rejeté, la trame retrouvée.
    const BsbFrame get      = BsbFrame::make_get(0x42, 0x00, fields::OUTDOOR_TEMP);
    const uint8_t  glitch[] = {0xDC, 0x8A};

    BsbFrameParser parser;
    BsbFrame       last;
    TEST_ASSERT_EQUAL_INT(0, feed_stream(parser, glitch, sizeof(glitch), last));
    TEST_ASSERT_EQUAL_INT(1, feed_stream(parser, get.raw, get.raw_len, last));
    TEST_ASSERT_EQUAL_HEX8(0x42, last.src_addr);
    TEST_ASSERT_EQUAL_HEX32(fields::OUTDOOR_TEMP, last.field_id);
    TEST_ASSERT_EQUAL_UINT32(0, parser.crc_error_count()); // Aucune trame complète n'a été rejetée
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
// 9. Tests des tables de codes statut Siemens (8000 / 8001 / 8003 / 8006)
// ============================================================================
void test_french_status_translations(void) {
    TEST_ASSERT_EQUAL_STRING("Compresseur 1 en marche", get_hp_status_text_fr(46));
    TEST_ASSERT_EQUAL_STRING("Pas de demande", get_hp_status_text_fr(51));
    TEST_ASSERT_EQUAL_STRING("Degivrage actif", get_hp_status_text_fr(125));
    TEST_ASSERT_EQUAL_STRING("Compresseur verrouille", get_hp_status_text_fr(258)); // Code sur 16 bits
    TEST_ASSERT_EQUAL_STRING("Chauffage confort", get_zone_status_text_fr(114));
    TEST_ASSERT_EQUAL_STRING("Limitation temperature ambiante", get_zone_status_text_fr(122));
    TEST_ASSERT_EQUAL_STRING("Restreint, priorite ECS", get_zone_status_text_fr(104));
    TEST_ASSERT_EQUAL_STRING("En charge, consigne nominale", get_dhw_status_text_fr(96));
    TEST_ASSERT_EQUAL_STRING("Charge terminee, temp. nominale", get_dhw_status_text_fr(99));
    // Codes non répertoriés : libellé générique, jamais de pointeur nul
    TEST_ASSERT_EQUAL_STRING("Autre etat PAC", get_hp_status_text_fr(999));
    TEST_ASSERT_EQUAL_STRING("Autre etat circuit chauffage", get_zone_status_text_fr(999));
    TEST_ASSERT_EQUAL_STRING("Autre etat ECS", get_dhw_status_text_fr(999));
}

void test_status_classification_follows_siemens_tables(void) {
    // Circuits de chauffage : seuls les états de demande réelle comptent comme « chauffe »
    TEST_ASSERT_TRUE(ZoneActivity::Heating == classify_zone_status(114));    // Chauffage confort
    TEST_ASSERT_TRUE(ZoneActivity::Heating == classify_zone_status(116));    // Chauffage réduit
    TEST_ASSERT_TRUE(ZoneActivity::Heating == classify_zone_status(101));    // Hors-gel ambiance actif
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(122));       // Limitation temp. ambiante
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(118));       // Régime d'été
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(119));       // Éco journalier
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(120));       // Abaissement réduit
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(17));        // Post-circulation
    TEST_ASSERT_TRUE(ZoneActivity::Idle == classify_zone_status(25));        // Arrêt
    TEST_ASSERT_TRUE(ZoneActivity::Restricted == classify_zone_status(104)); // Priorité ECS
    TEST_ASSERT_TRUE(ZoneActivity::Unknown == classify_zone_status(0));
    TEST_ASSERT_TRUE(ZoneActivity::Unknown == classify_zone_status(999));

    // ECS : 96 = charge en cours (et non « charge prête »), 99 = ballon chargé
    TEST_ASSERT_TRUE(DhwActivity::Charging == classify_dhw_status(96));
    TEST_ASSERT_TRUE(DhwActivity::Charging == classify_dhw_status(97));
    TEST_ASSERT_TRUE(DhwActivity::Charging == classify_dhw_status(69));
    TEST_ASSERT_TRUE(DhwActivity::ChargingElectric == classify_dhw_status(88));
    TEST_ASSERT_TRUE(DhwActivity::Idle == classify_dhw_status(99));
    TEST_ASSERT_TRUE(DhwActivity::Idle == classify_dhw_status(100));
    TEST_ASSERT_TRUE(DhwActivity::Idle == classify_dhw_status(80));
    TEST_ASSERT_TRUE(DhwActivity::Idle == classify_dhw_status(25));
    TEST_ASSERT_TRUE(DhwActivity::Unknown == classify_dhw_status(0));

    // Pompe à chaleur
    TEST_ASSERT_TRUE(HeatPumpActivity::Compressor == classify_hp_status(46));
    TEST_ASSERT_TRUE(HeatPumpActivity::CompressorAndElectric == classify_hp_status(44));
    TEST_ASSERT_TRUE(HeatPumpActivity::ElectricOnly == classify_hp_status(197));
    TEST_ASSERT_TRUE(HeatPumpActivity::Stopped == classify_hp_status(35)); // Temps d'arrêt mini compresseur
    TEST_ASSERT_TRUE(HeatPumpActivity::Stopped == classify_hp_status(50)); // Libérée, compresseur prêt
    TEST_ASSERT_TRUE(HeatPumpActivity::Stopped == classify_hp_status(51)); // Pas de demande
    TEST_ASSERT_TRUE(HeatPumpActivity::Defrost == classify_hp_status(125));
    TEST_ASSERT_TRUE(HeatPumpActivity::Cooling == classify_hp_status(127));
}

void test_decode_status_code_on_16_bits(void) {
    // Code statut PAC étendu 258 (0x0102) renvoyé sur 2 octets : ne doit pas être tronqué à 8 bits
    const uint8_t pl[] = {0x00, 0x01, 0x02};
    DecodedValue v = build_ret_frame(fields::HP_STATUS_CODE, pl, sizeof(pl)).decode();
    TEST_ASSERT_TRUE(v.valid);
    TEST_ASSERT_EQUAL_INT32(258, v.int_val);
}

// ============================================================================
// 10. État de chauffe réel des thermostats (scénarios relevés sur une Alféa RVS21 bi-zone)
// ============================================================================
static HeatingInputs make_two_zone_dhw_inputs(void) {
    HeatingInputs in;
    in.has_zone1            = true;
    in.has_zone2            = true;
    in.has_dhw              = true;
    in.has_compressor_state = true;
    in.has_compressor_mod   = true;
    in.has_backup_state     = true;
    in.has_backup_mod       = true;
    in.zone2_has_pump_state = true;
    in.zone1_status         = 114; // Chauffage confort
    in.zone2_status         = 114;
    in.dhw_status           = 99;  // Ballon chargé
    in.hp_status            = 51;  // Pas de demande
    return in;
}

void test_heating_zone_in_room_limitation_is_idle_while_other_zone_heats(void) {
    // Cas signalé : Zone 2 en « Limitation température ambiante » (122, pompe Q6 arrêtée) alors que le
    // compresseur tourne à 100 % pour la Zone 1 -> seule la Zone 1 chauffe.
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    in.zone2_status   = 122;

    const HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.heat_source_on);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.zone1.demand);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_EQUAL_UINT8(0, out.zone2.demand);
    TEST_ASSERT_FALSE(out.dhw.heating);
}

void test_heating_both_zones_follow_compressor_cycles(void) {
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 45;
    in.hp_status      = 46;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_TRUE(out.zone2.heating);
    TEST_ASSERT_EQUAL_UINT8(45, out.zone1.demand);
    TEST_ASSERT_EQUAL_UINT8(45, out.zone2.demand);

    // Compresseur à l'arrêt entre deux cycles (« Libérée, compresseur prêt ») : plus aucune chauffe,
    // même si les circuits restent en régime confort.
    in.compressor_on  = false;
    in.compressor_mod = 0;
    in.hp_status      = 50;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.heat_source_on);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_EQUAL_UINT8(0, out.zone1.demand);
}

void test_heating_dhw_charge_takes_priority_over_zones(void) {
    // Charge ECS : statut ECS 96, Zone 1 « Restreint, priorité ECS » (104), Zone 2 limitée (122)
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    in.dhw_status     = 96;
    in.zone1_status   = 104;
    in.zone2_status   = 122;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.dhw.demand);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);

    // Transition : le statut de zone n'est pas encore relu (toujours 114) mais la PAC charge déjà le ballon
    in.zone1_status = 114;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone1.heating);

    // Ballon chargé (99) : la chauffe revient aux zones
    in.dhw_status = 99;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.dhw.heating);
    TEST_ASSERT_TRUE(out.zone1.heating);
}

void test_heating_dhw_electric_charge_is_independent_of_compressor(void) {
    // Charge ECS par la résistance du ballon pendant que le compresseur chauffe la Zone 1
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 60;
    in.hp_status      = 46;
    in.dhw_status     = 88;
    in.zone2_status   = 122;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.dhw.demand);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(60, out.zone1.demand);

    // Même charge électrique, compresseur à l'arrêt
    in.compressor_on  = false;
    in.compressor_mod = 0;
    in.hp_status      = 51;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone1.heating);
}

void test_heating_defrost_and_cooling_are_not_heating(void) {
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 80;

    in.hp_status = 125; // Dégivrage actif : le compresseur tourne mais ne chauffe pas l'installation
    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.heat_source_on);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);

    in.hp_status = 209; // Compresseur 1 en marche, rafraîchissement
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating);

    // Dégivrage avec appoint électrique actif : l'appoint, lui, chauffe bien les circuits
    in.hp_status  = 125;
    in.backup_on  = true;
    in.backup_mod = 0;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.heat_source_on);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.zone1.demand); // Appoint sans modulation connue : 100 %
}

void test_heating_demand_is_never_zero_while_heating(void) {
    // ZHA déduit l'action CVC de PIHeatingDemand (> 0) : une chauffe sans modulation lue vaut 100 %
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 0; // Régulateur renvoyant « --- » pour la modulation
    in.hp_status      = 46;

    const HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.zone1.demand);
}

void test_heating_fallbacks_without_status_codes(void) {
    // Régulateur sans codes statut de zone : repli sur la pompe du circuit, puis sur le mode
    HeatingInputs in;
    in.has_zone1            = true;
    in.has_zone2            = true;
    in.has_compressor_state = true;
    in.compressor_on        = true;
    in.zone2_has_pump_state = true;
    in.zone2_pump_on        = false;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone1.heating);  // Ni statut ni pompe : mode actif -> chauffe avec le générateur
    TEST_ASSERT_FALSE(out.zone2.heating); // Pompe Q6 arrêtée

    in.zone2_pump_on         = true;
    in.zone1_protection_mode = true;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating); // Zone 1 en mode hors-gel
    TEST_ASSERT_TRUE(out.zone2.heating);

    // Sans état compresseur (8400) ni modulation (8413) : le code statut PAC sert de repli
    HeatingInputs minimal;
    minimal.has_zone1    = true;
    minimal.zone1_status = 114;
    minimal.hp_status    = 46; // Compresseur 1 en marche
    TEST_ASSERT_TRUE(compute_heating_state(minimal).zone1.heating);
    minimal.hp_status = 51;    // Pas de demande
    TEST_ASSERT_FALSE(compute_heating_state(minimal).zone1.heating);
    minimal.hp_status = 197;   // Appoint électrique en marche
    TEST_ASSERT_TRUE(compute_heating_state(minimal).zone1.heating);
}

void test_heating_zero_modulation_does_not_hide_status_code(void) {
    // Régulateur sans état matériel du compresseur (8400 absent) dont la modulation (8413) répond « --- »
    // en permanence, donc lue 0 % : le code statut PAC doit quand même faire foi.
    HeatingInputs in;
    in.has_zone1          = true;
    in.zone1_status       = 114;
    in.has_compressor_mod = true;
    in.compressor_mod     = 0;
    in.hp_status          = 46; // Compresseur 1 en marche
    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.heat_source_on);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.zone1.demand); // Modulation inconnue : demande pleine

    in.hp_status = 51; // Pas de demande
    TEST_ASSERT_FALSE(compute_heating_state(in).zone1.heating);

    // Même règle pour l'appoint : modulation fantôme à 0 %, état matériel absent, statut « appoint en marche »
    HeatingInputs backup;
    backup.has_zone1      = true;
    backup.zone1_status   = 114;
    backup.has_backup_mod = true;
    backup.backup_mod     = 0;
    backup.hp_status      = 197;
    TEST_ASSERT_TRUE(compute_heating_state(backup).zone1.heating);

    // L'état matériel, lorsqu'il existe, reste prioritaire sur le code statut
    HeatingInputs hard;
    hard.has_zone1            = true;
    hard.zone1_status         = 114;
    hard.has_compressor_state = true;
    hard.compressor_on        = false;
    hard.has_backup_state     = true;
    hard.backup_on            = false;
    hard.hp_status            = 46;
    TEST_ASSERT_FALSE(compute_heating_state(hard).zone1.heating);
}

void test_heating_requires_flow_warmer_than_return(void) {
    // Relevé terrain (Alféa RVS21, 2026-11-13 15:15-17:15) : le régulateur annonce le compresseur en marche
    // à 100 % (8400 = ON, 8413 = 100, 8006 = 46) alors que la PAC est à l'arrêt (13 W au compteur), circuit
    // radiateurs fermé : départ 22 C, retour 27 C. Aucune chaleur n'atteint la Zone 1.
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    in.zone2_status   = 122;
    in.has_hp_temps   = true;
    in.hp_flow_c      = 22.0f;
    in.hp_return_c    = 27.0f;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.heat_source_on);        // Ce que dit le régulateur
    TEST_ASSERT_FALSE(out.heating_circuits_fed); // Ce que disent les sondes
    TEST_ASSERT_TRUE(out.zone_heating_refuted);  // La Zone 1 demandait : la garde lui retire la chauffe
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(0, out.zone1.demand);
    TEST_ASSERT_FALSE(out.zone2.heating);

    // Fonctionnement réel relevé le même jour (10:30-12:10) : départ 33 C, retour 19 C -> la Zone 1 chauffe
    in.compressor_mod = 45;
    in.hp_flow_c      = 33.0f;
    in.hp_return_c    = 19.3f;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.heating_circuits_fed);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_EQUAL_UINT8(45, out.zone1.demand);

    // Écart faiblement négatif (tolérance des sondes, démarrage) : la chauffe n'est pas masquée
    in.hp_flow_c   = 25.0f;
    in.hp_return_c = 25.6f;
    TEST_ASSERT_TRUE(compute_heating_state(in).zone1.heating);

    // Charge ECS : départ et retour sont quasi égaux (retour parfois 1 K au-dessus), la garde ne s'applique pas
    in.dhw_status     = 96;
    in.zone1_status   = 104;
    in.compressor_mod = 100;
    in.hp_flow_c      = 54.2f;
    in.hp_return_c    = 55.2f;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_EQUAL_UINT8(100, out.dhw.demand);
    TEST_ASSERT_FALSE(out.zone1.heating);

    // Sondes indisponibles : seul le régulateur fait foi, comme avant
    HeatingInputs no_temps = make_two_zone_dhw_inputs();
    no_temps.compressor_on  = true;
    no_temps.compressor_mod = 100;
    no_temps.hp_status      = 46;
    no_temps.hp_flow_c      = 22.0f;
    no_temps.hp_return_c    = 27.0f; // Ignorés : has_hp_temps est faux
    TEST_ASSERT_TRUE(compute_heating_state(no_temps).zone1.heating);
}

void test_heating_unread_status_code_is_not_heating(void) {
    // Au démarrage (ou après un silence du bus), l'état compresseur est relu avant le code statut de la
    // zone : tant que ce code vaut 0, rien n'est affirmé, même si la pompe du circuit tourne.
    HeatingInputs in;
    in.has_zone1             = true;
    in.has_zone2             = true;
    in.has_compressor_state  = true;
    in.compressor_on         = true;
    in.zone1_has_status_code = true;
    in.zone2_has_status_code = true;
    in.zone2_has_pump_state  = true;
    in.zone2_pump_on         = true;
    in.zone1_status          = 0;
    in.zone2_status          = 0;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);

    in.zone2_status = 114; // Code relu : chauffage confort
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_TRUE(out.zone2.heating);
}

void test_heating_unlisted_status_code_follows_circuit_pump(void) {
    // Relevé terrain (2026-11-14 14:23) : consigne temporaire de 23,5 C réglée sur l'afficheur de la PAC pour
    // le plancher chauffant. Le régulateur donne alors à la zone 2 le code 255, absent des tables de
    // référence, pendant que la pompe Q6 tourne et que le compresseur chauffe (départ 34,4 C, retour 23,8 C).
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.zone1_has_status_code = true;
    in.zone2_has_status_code = true;
    in.compressor_on  = true;
    in.compressor_mod = 82;
    in.hp_status      = 46;
    in.zone1_status   = 122;
    in.zone2_status   = 255;
    in.zone2_pump_on  = true;
    in.has_hp_temps   = true;
    in.hp_flow_c      = 34.4f;
    in.hp_return_c    = 23.8f;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone2.heating);
    TEST_ASSERT_EQUAL_UINT8(82, out.zone2.demand);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::Pump), static_cast<int>(out.zone2_evidence));
    TEST_ASSERT_FALSE(out.zone1.heating); // Zone 1 en limitation d'ambiance : son code répertorié fait foi
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::StatusCode), static_cast<int>(out.zone1_evidence));

    // Le régulateur coupe la pompe du circuit : la zone n'est plus alimentée, quel que soit son code
    in.zone2_pump_on = false;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::Pump), static_cast<int>(out.zone2_evidence));

    // Pompe en marche mais compresseur à l'arrêt entre deux cycles : pas de chauffe
    in.zone2_pump_on  = true;
    in.compressor_on  = false;
    in.compressor_mod = 0;
    in.hp_status      = 35;
    TEST_ASSERT_FALSE(compute_heating_state(in).zone2.heating);

    // Les autres règles s'appliquent comme pour un code répertorié : priorité à la charge du ballon...
    in.compressor_on  = true;
    in.compressor_mod = 82;
    in.hp_status      = 46;
    in.dhw_status     = 96;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);

    // ... et garde hydraulique (retour plus chaud que le départ)
    in.dhw_status  = 99;
    in.hp_flow_c   = 22.0f;
    in.hp_return_c = 27.0f;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_TRUE(out.zone_heating_refuted);
}

void test_heating_unlisted_status_code_without_pump_asserts_nothing(void) {
    // Même code inconnu sur un circuit dont le régulateur n'expose pas la pompe (zone 1 directe de l'Alféa) :
    // rien ne permet de savoir si la zone est alimentée, elle n'est donc pas donnée en chauffe.
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.zone1_has_status_code = true;
    in.zone2_has_status_code = true;
    in.compressor_on  = true;
    in.compressor_mod = 60;
    in.hp_status      = 46;
    in.zone1_status   = 255;
    in.zone2_status   = 114;
    in.zone2_pump_on  = true;

    const HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::None), static_cast<int>(out.zone1_evidence));
    TEST_ASSERT_TRUE(out.zone2.heating);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::StatusCode), static_cast<int>(out.zone2_evidence));

    // Un code pas encore relu (0) n'est pas un code inconnu : rien n'est affirmé, même pompe en marche
    HeatingInputs boot = in;
    boot.zone2_status = 0;
    const HeatingOutputs at_boot = compute_heating_state(boot);
    TEST_ASSERT_FALSE(at_boot.zone2.heating);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::None), static_cast<int>(at_boot.zone2_evidence));

    // Régulateur sans code statut du tout : le repli historique sur le mode de la zone est conservé
    HeatingInputs legacy;
    legacy.has_zone1            = true;
    legacy.has_compressor_state = true;
    legacy.compressor_on        = true;
    const HeatingOutputs legacy_out = compute_heating_state(legacy);
    TEST_ASSERT_TRUE(legacy_out.zone1.heating);
    TEST_ASSERT_EQUAL(static_cast<int>(ZoneEvidence::Mode), static_cast<int>(legacy_out.zone1_evidence));
}

void test_heating_absent_circuits_never_heat(void) {
    HeatingInputs in;
    in.has_compressor_state = true;
    in.compressor_on        = true;
    in.zone1_status         = 114;
    in.zone2_status         = 114;
    in.dhw_status           = 96;

    const HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_FALSE(out.dhw.heating);
}

void test_heating_guard_only_refutes_an_actual_zone_demand(void) {
    // Relevé terrain (2026-11-14 10:27) : reprise d'une charge ECS après dégivrage, départ 22,4 C et retour
    // 25,1 C. Aucune zone ne demandait (104 = priorité ECS, 122 = limitation d'ambiance) : la garde
    // hydraulique ne retire rien à personne et ne doit donc rien signaler.
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    in.zone1_status   = 104;
    in.zone2_status   = 122;
    in.dhw_status     = 96;
    in.has_hp_temps   = true;
    in.hp_flow_c      = 22.4f;
    in.hp_return_c    = 25.1f;

    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.heating_circuits_fed);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone1.heating);

    // Même écart inversé, ballon chargé et générateur à l'arrêt : il n'y a rien à démentir non plus
    in.compressor_on  = false;
    in.compressor_mod = 0;
    in.hp_status      = 51;
    in.dhw_status     = 99;
    in.zone1_status   = 114;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.heat_source_on);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);

    // Générateur annoncé en marche pour une zone en demande : là, la garde dément le régulateur
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone_heating_refuted);
    TEST_ASSERT_FALSE(out.zone1.heating);
}

void test_heating_start_is_published_only_once_confirmed(void) {
    const ThermostatAction idle{};
    ThermostatAction heating;
    heating.heating = true;
    heating.demand  = 45;

    HeatingConfirmation filter;
    TEST_ASSERT_FALSE(filter.update(idle, 1000).heating);
    TEST_ASSERT_FALSE(filter.pending());

    // Relevé terrain (2026-11-14 10:26:48) : Zone 1 calculée en chauffe pendant 2,7 s, le temps que ses
    // codes statut soient relus. Rien ne doit être publié.
    TEST_ASSERT_FALSE(filter.update(heating, 10000).heating);
    TEST_ASSERT_TRUE(filter.pending());
    TEST_ASSERT_FALSE(filter.update(heating, 12700).heating);
    TEST_ASSERT_FALSE(filter.update(idle, 12701).heating);
    TEST_ASSERT_FALSE(filter.pending());

    // Relevé terrain (2026-11-14 03:25:02) : 20 s de chauffe calculée après un dégivrage, jusqu'à la
    // relecture des sondes départ / retour. Le délai repart de zéro à chaque interruption.
    TEST_ASSERT_FALSE(filter.update(heating, 20000).heating);
    TEST_ASSERT_FALSE(filter.update(heating, 39999).heating);
    TEST_ASSERT_FALSE(filter.update(idle, 40000).heating);
    TEST_ASSERT_FALSE(filter.update(heating, 41000).heating);
    TEST_ASSERT_FALSE(filter.update(heating, 41000 + HEATING_CONFIRM_MS - 1).heating);

    // Chauffe maintenue pendant toute la durée de confirmation : publiée, avec la demande du moment
    ThermostatAction published = filter.update(heating, 41000 + HEATING_CONFIRM_MS);
    TEST_ASSERT_TRUE(published.heating);
    TEST_ASSERT_EQUAL_UINT8(45, published.demand);
    TEST_ASSERT_FALSE(filter.pending());

    // Une fois confirmée, la demande suit le calcul sans délai
    heating.demand = 80;
    published = filter.update(heating, 41000 + HEATING_CONFIRM_MS + 5000);
    TEST_ASSERT_TRUE(published.heating);
    TEST_ASSERT_EQUAL_UINT8(80, published.demand);

    // L'arrêt, lui, est immédiat et remet la demande à zéro
    published = filter.update(idle, 41000 + HEATING_CONFIRM_MS + 5001);
    TEST_ASSERT_FALSE(published.heating);
    TEST_ASSERT_EQUAL_UINT8(0, published.demand);

    // La reprise suivante doit être confirmée à son tour
    TEST_ASSERT_FALSE(filter.update(heating, 200000).heating);
    TEST_ASSERT_TRUE(filter.update(heating, 200000 + HEATING_CONFIRM_MS).heating);
}

void test_heating_first_verdict_after_boot_is_published_as_is(void) {
    ThermostatAction heating;
    heating.heating = true;
    heating.demand  = 32;

    // Pont redémarré en pleine chauffe (mise à jour OTA, coupure) : pas d'arrêt fictif de 30 s
    HeatingConfirmation rebooted_while_heating;
    const ThermostatAction first = rebooted_while_heating.update(heating, 4000);
    TEST_ASSERT_TRUE(first.heating);
    TEST_ASSERT_EQUAL_UINT8(32, first.demand);

    // Pont redémarré à l'arrêt : la première mise en chauffe suit la règle commune
    HeatingConfirmation rebooted_while_idle;
    TEST_ASSERT_FALSE(rebooted_while_idle.update(ThermostatAction{}, 4000).heating);
    TEST_ASSERT_FALSE(rebooted_while_idle.update(heating, 5000).heating);
    TEST_ASSERT_TRUE(rebooted_while_idle.update(heating, 5000 + HEATING_CONFIRM_MS).heating);
}

void test_unfed_zone_is_reported_only_when_it_outlasts_a_normal_start(void) {
    // Relevé terrain (2026-11-14 22:47-22:49) : mise en route après un arrêt prolongé, zone 1 en demande
    // (114), compresseur annoncé en marche, départ 15,6 C et retour 18,5 C. La garde hydraulique tient la
    // zone pour non alimentée...
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 16;
    in.hp_status      = 46;
    in.zone1_status   = 114;
    in.zone2_status   = 122;
    in.has_hp_temps   = true;
    in.hp_flow_c      = 15.6f;
    in.hp_return_c    = 18.5f;
    TEST_ASSERT_TRUE(compute_heating_state(in).zone_heating_refuted);

    // ... pendant une minute environ : c'est une mise en route, pas une anomalie à signaler
    UnfedZoneWatch watch;
    const uint64_t t0 = 1087000;
    TEST_ASSERT_FALSE(watch.update(true, t0));
    TEST_ASSERT_TRUE(watch.in_grace_period());
    TEST_ASSERT_FALSE(watch.alerted());
    TEST_ASSERT_FALSE(watch.update(true, t0 + 40000));
    TEST_ASSERT_TRUE(watch.in_grace_period());

    // Le départ repasse au-dessus du retour (16,5 C / 15,7 C) : la garde se lève, rien n'a été signalé
    in.compressor_mod = 60;
    in.hp_flow_c      = 16.5f;
    in.hp_return_c    = 15.7f;
    const HeatingOutputs started = compute_heating_state(in);
    TEST_ASSERT_FALSE(started.zone_heating_refuted);
    TEST_ASSERT_TRUE(started.zone1.heating);
    TEST_ASSERT_FALSE(watch.update(started.zone_heating_refuted, t0 + 70000));
    TEST_ASSERT_FALSE(watch.in_grace_period());
    TEST_ASSERT_FALSE(watch.alerted());

    // Anomalie réelle (relevé du 2026-11-13) : générateur annoncé en marche pendant des heures, retour plus
    // chaud que le départ. Elle est signalée une fois, quand elle a duré plus qu'une mise en route.
    const uint64_t t1 = 5000000;
    TEST_ASSERT_FALSE(watch.update(true, t1));
    TEST_ASSERT_FALSE(watch.update(true, t1 + UNFED_ZONE_ALERT_MS - 1));
    TEST_ASSERT_TRUE(watch.in_grace_period());
    TEST_ASSERT_TRUE(watch.update(true, t1 + UNFED_ZONE_ALERT_MS));
    TEST_ASSERT_TRUE(watch.alerted());
    TEST_ASSERT_FALSE(watch.in_grace_period());
    TEST_ASSERT_FALSE(watch.update(true, t1 + UNFED_ZONE_ALERT_MS + 30000)); // Pas de rappel
    TEST_ASSERT_FALSE(watch.update(true, t1 + 10ULL * UNFED_ZONE_ALERT_MS));
    TEST_ASSERT_TRUE(watch.alerted());

    // Fin de l'épisode : tout est oublié, le suivant sera signalé à son tour s'il dure
    TEST_ASSERT_FALSE(watch.update(false, t1 + 11ULL * UNFED_ZONE_ALERT_MS));
    TEST_ASSERT_FALSE(watch.alerted());
    const uint64_t t2 = t1 + 12ULL * UNFED_ZONE_ALERT_MS;
    TEST_ASSERT_FALSE(watch.update(true, t2));
    TEST_ASSERT_TRUE(watch.update(true, t2 + UNFED_ZONE_ALERT_MS));

    // Une horloge qui reculerait ne déclenche rien
    UnfedZoneWatch clock_back;
    TEST_ASSERT_FALSE(clock_back.update(true, 9000000));
    TEST_ASSERT_FALSE(clock_back.update(true, 1000));
    TEST_ASSERT_FALSE(clock_back.alerted());
}

void test_heating_resumes_after_a_dhw_charge_once_the_guard_lifts(void) {
    // Relevé terrain (2026-11-15 11:10-11:11) : fin d'une charge ECS de 42 min, compresseur en marche à 100 %.
    // Tant que la charge dure, un retour lu 1 K au-dessus du départ ne dément rien : la chaleur va au ballon.
    HeatingInputs in = make_two_zone_dhw_inputs();
    in.compressor_on  = true;
    in.compressor_mod = 100;
    in.hp_status      = 46;
    in.zone1_status   = 104; // Restreint, priorité ECS
    in.zone2_status   = 122; // Limitation température ambiante
    in.dhw_status     = 96;  // En charge, consigne nominale
    in.has_hp_temps   = true;
    in.hp_flow_c      = 53.9f;
    in.hp_return_c    = 54.9f;
    HeatingOutputs out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);

    const uint64_t      t = 11476338; // Horloge du pont à la fin de la charge
    HeatingConfirmation zone1;
    UnfedZoneWatch      watch;
    TEST_ASSERT_FALSE(zone1.update(out.zone1, t - 1000).heating);

    // 11:10:43 : la charge se termine (17) et la zone 1 redevient demandeuse (114). Écart inversé de 0,9 K,
    // sous le seuil de la garde : la chauffe de la zone est calculée et attend sa confirmation.
    in.dhw_status   = 17;
    in.zone1_status = 114;
    in.hp_flow_c    = 54.1f;
    in.hp_return_c  = 55.0f;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.dhw.heating);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_FALSE(zone1.update(out.zone1, t).heating);
    TEST_ASSERT_TRUE(zone1.pending());
    TEST_ASSERT_FALSE(watch.update(out.zone_heating_refuted, t));

    // 0,8 s plus tard le retour est relu à 55,2 C : la garde agit et la confirmation en cours est abandonnée
    in.hp_return_c = 55.2f;
    out = compute_heating_state(in);
    TEST_ASSERT_TRUE(out.zone_heating_refuted);
    TEST_ASSERT_FALSE(out.zone1.heating);
    TEST_ASSERT_FALSE(zone1.update(out.zone1, t + 780).heating);
    TEST_ASSERT_FALSE(zone1.pending());
    TEST_ASSERT_FALSE(watch.update(out.zone_heating_refuted, t + 780));
    TEST_ASSERT_TRUE(watch.in_grace_period());

    // 16 s après, l'eau des zones revient et la garde se lève sans avoir rien signalé (relevé suivant :
    // départ 54,2 C, retour 54,9 C). La chauffe est publiée une fois confirmée, pour la seule zone en demande.
    in.hp_flow_c   = 54.2f;
    in.hp_return_c = 54.9f;
    out = compute_heating_state(in);
    TEST_ASSERT_FALSE(out.zone_heating_refuted);
    TEST_ASSERT_TRUE(out.zone1.heating);
    TEST_ASSERT_FALSE(out.zone2.heating);
    TEST_ASSERT_FALSE(watch.update(out.zone_heating_refuted, t + 17000));
    TEST_ASSERT_FALSE(watch.in_grace_period());
    TEST_ASSERT_FALSE(watch.alerted());
    TEST_ASSERT_FALSE(zone1.update(out.zone1, t + 17000).heating);
    TEST_ASSERT_TRUE(zone1.pending());
    TEST_ASSERT_FALSE(zone1.update(out.zone1, t + 17000 + HEATING_CONFIRM_MS - 1).heating);
    const ThermostatAction published = zone1.update(out.zone1, t + 17000 + HEATING_CONFIRM_MS);
    TEST_ASSERT_TRUE(published.heating);
    TEST_ASSERT_EQUAL_UINT8(100, published.demand);
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
    RUN_TEST(test_parser_finds_a_frame_swallowed_whole_by_a_corrupted_length);
    RUN_TEST(test_parser_is_not_fooled_by_a_data_byte_that_looks_like_a_frame_start);
    RUN_TEST(test_parser_finds_a_frame_starting_inside_a_rejected_header);
    RUN_TEST(test_decode_controller_model_string_strips_null);
    RUN_TEST(test_decode_binary_states_and_modulations);
    RUN_TEST(test_autodiscovery_sentinel_filtering);
    RUN_TEST(test_french_status_translations);
    RUN_TEST(test_status_classification_follows_siemens_tables);
    RUN_TEST(test_decode_status_code_on_16_bits);
    RUN_TEST(test_heating_zone_in_room_limitation_is_idle_while_other_zone_heats);
    RUN_TEST(test_heating_both_zones_follow_compressor_cycles);
    RUN_TEST(test_heating_dhw_charge_takes_priority_over_zones);
    RUN_TEST(test_heating_dhw_electric_charge_is_independent_of_compressor);
    RUN_TEST(test_heating_defrost_and_cooling_are_not_heating);
    RUN_TEST(test_heating_demand_is_never_zero_while_heating);
    RUN_TEST(test_heating_fallbacks_without_status_codes);
    RUN_TEST(test_heating_requires_flow_warmer_than_return);
    RUN_TEST(test_heating_unread_status_code_is_not_heating);
    RUN_TEST(test_heating_unlisted_status_code_follows_circuit_pump);
    RUN_TEST(test_heating_unlisted_status_code_without_pump_asserts_nothing);
    RUN_TEST(test_heating_absent_circuits_never_heat);
    RUN_TEST(test_heating_zero_modulation_does_not_hide_status_code);
    RUN_TEST(test_heating_guard_only_refutes_an_actual_zone_demand);
    RUN_TEST(test_heating_start_is_published_only_once_confirmed);
    RUN_TEST(test_heating_first_verdict_after_boot_is_published_as_is);
    RUN_TEST(test_unfed_zone_is_reported_only_when_it_outlasts_a_normal_start);
    RUN_TEST(test_heating_resumes_after_a_dhw_charge_once_the_guard_lifts);
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

#pragma once

#include <cstdint>
#include <cstddef>
#include "bsb_types.h"

namespace bsb {

/**
 * @brief Structure représentant une trame BSB complète (sans allocation dynamique).
 *
 * Format sur le fil (après inversion bit-à-bit ^ 0xFF) :
 *   [0]      0xDC (Octet de synchronisation Start-Of-Frame)
 *   [1]      src_addr | 0x80
 *   [2]      dst_addr
 *   [3]      length (longueur totale trame = 11 + payload_len)
 *   [4]      command (GET=0x06, RET=0x07, SET=0x03, ACK=0x04, NACK=0x05, INF=0x02)
 *   [5..8]   field_id (octets 5 et 6 inversés pour GET/SET/INF, ordre normal pour RET/ACK/NACK)
 *   [9..N-3] payload (0 à 32 octets)
 *   [N-2..N-1] CRC-16 CCITT XMODEM big-endian sur les octets [0..N-3]
 */
struct BsbFrame {
    static constexpr uint8_t SOF_BYTE        = 0xDC;
    static constexpr size_t  HEADER_NO_PL    = 11; // 1(SOF) + 1(SRC) + 1(DST) + 1(LEN) + 1(CMD) + 4(FIELD) + 2(CRC)
    static constexpr size_t  MAX_PAYLOAD_LEN = 32;
    static constexpr size_t  MAX_FRAME_LEN   = HEADER_NO_PL + MAX_PAYLOAD_LEN;

    uint8_t  src_addr    = 0;
    uint8_t  dst_addr    = 0;
    Command  command     = Command::None;
    uint32_t field_id    = 0;
    uint8_t  payload[MAX_PAYLOAD_LEN] = {0};
    uint8_t  payload_len = 0;
    uint16_t crc         = 0;

    uint8_t  raw[MAX_FRAME_LEN] = {0};
    uint8_t  raw_len     = 0;

    /**
     * @brief Calcule le CRC-16 CCITT (XMODEM, poly=0x1021, init=0x0000).
     */
    static uint16_t compute_crc16(const uint8_t* data, size_t len);

    /**
     * @brief Échange les deux premiers octets (MSB et MSB-1) d'un field_id.
     */
    static uint32_t swap_field_msb(uint32_t field_id) {
        return ((field_id & 0x00FF0000UL) << 8) |
               ((field_id & 0xFF000000UL) >> 8) |
               (field_id & 0x0000FFFFUL);
    }

    /**
     * @brief Sérialise la trame dans `raw[]` et calcule le CRC.
     */
    void serialize();

    /**
     * @brief Fabrique une trame GET (lecture d'un paramètre).
     */
    static BsbFrame make_get(uint8_t src, uint8_t dst, uint32_t field_id);

    /**
     * @brief Fabrique une trame SET pour un entier 8 bits / mode (uint8_t).
     */
    static BsbFrame make_set_uint8(uint8_t src, uint8_t dst, uint32_t field_id, uint8_t value, uint8_t enable_byte = 0x01);

    /**
     * @brief Fabrique une trame SET pour une température (°C, encodée en int16 * 64).
     */
    static BsbFrame make_set_temperature(uint8_t src, uint8_t dst, uint32_t field_id, float temp_c, uint8_t enable_byte = 0x01);

    /**
     * @brief Fabrique une trame INF (broadcast 0x7F) pour injecter une température d'ambiance externe.
     */
    static BsbFrame make_inf_room_temperature(uint8_t src, uint32_t field_id, float temp_c);

    /**
     * @brief Décode la charge utile (payload) de la trame selon le type de donnée attendu.
     */
    DecodedValue decode(ValueType expected_type = ValueType::Unknown) const;

    /**
     * @brief Formate la trame en chaîne lisible pour le journal de diagnostic.
     */
    void format_debug(char* out_buf, size_t out_size) const;
};

/**
 * @brief Automate à états finis robuste pour le décodage flux octet-par-octet du bus BSB.
 *        Protégé contre les trames tronquées, les dépassements de tampon et le bruit sur le bus.
 */
class BsbFrameParser {
public:
    BsbFrameParser() = default;

    void reset();

    /**
     * @brief Ingère un octet déjà inversé (byte_uart ^ 0xFF).
     * @param byte Octet logique BSB
     * @param out_frame Rempli si une trame complète avec CRC valide vient d'être reçue
     * @return true lorsqu'une trame complète et valide est prête dans `out_frame`
     */
    bool feed(uint8_t byte, BsbFrame& out_frame);

private:
    enum class State : uint8_t {
        WaitSof,
        SrcAddr,
        DstAddr,
        Length,
        CommandByte,
        Field0,
        Field1,
        Field2,
        Field3,
        Payload,
        CrcMsb,
        CrcLsb
    };

    State    state_        = State::WaitSof;
    BsbFrame working_      = {};
    uint8_t  expected_len_ = 0;
    uint8_t  payload_idx_  = 0;
};

} // namespace bsb

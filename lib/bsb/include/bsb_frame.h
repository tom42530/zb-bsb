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
 *   [5..8]   field_id (octets 5 et 6 inversés pour GET, SET et INF adressé à un appareil ;
 *            ordre normal pour RET/ACK/NACK et pour les INF diffusés en broadcast 0x7F)
 *   [9..N-3] payload (0 à 32 octets)
 *   [N-2..N-1] CRC-16 CCITT XMODEM big-endian sur les octets [0..N-3]
 */
struct BsbFrame {
    static constexpr uint8_t SOF_BYTE        = 0xDC;
    static constexpr uint8_t BROADCAST_ADDR  = 0x7F;
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
     * @brief Indique si les 2 premiers octets du field_id sont permutés sur le fil pour ce type de trame.
     *        (convention du protocole BSB : requêtes GET/SET, et INF adressé à un appareil précis)
     */
    static bool has_swapped_field_id(Command command, uint8_t dst_addr) {
        return command == Command::Get || command == Command::Set ||
               (command == Command::Inf && dst_addr != BROADCAST_ADDR);
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
     * @brief Fabrique une trame INF d'injection de température d'ambiance externe, adressée au régulateur
     *        (même trame qu'une sonde d'ambiance QAA : Prog 10000 = 0x2D3D0215, Prog 10001 = 0x2E3E0215,
     *        payload = [msb, lsb, 0x00]).
     */
    static BsbFrame make_inf_room_temperature(uint8_t src, uint8_t dst, uint32_t field_id, float temp_c);

    /**
     * @brief Indique si cette trame SET demande exactement la valeur que porte une réponse RET du régulateur.
     *
     * Sert à ne pas réécrire un paramètre qui a déjà la valeur voulue (chaque SET est mémorisé par le
     * régulateur). Ne vaut que pour une écriture ordinaire (octet d'activation 0x01) comparée à une
     * réponse exploitable (octet d'état 0x00) de même longueur : dans tous les autres cas le résultat
     * est false, et l'écriture doit avoir lieu.
     *
     * @param ret_payload      Charge utile de la réponse RET (octet d'état suivi de la valeur)
     * @param ret_payload_len  Longueur de cette charge utile
     */
    bool requests_value_of(const uint8_t* ret_payload, uint8_t ret_payload_len) const;

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
 *
 * Le flux n'a pas d'autre repère que l'octet 0xDC, qui peut aussi figurer dans les données. Quand une
 * trame est rejetée (en-tête impossible ou CRC faux), les octets qu'elle a absorbés ne sont donc pas
 * jetés : ils sont relus à partir du 0xDC suivant qu'ils contiennent. Sans cela, une trame amputée d'un
 * octet (octet perdu, émetteur qui cède le bus) se complète avec le début de la suivante et l'entraîne
 * dans son rejet : un seul octet manquant coûterait deux trames.
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
     *
     * Un appel rend au plus une trame. Si la relecture d'une trame rejetée en révélait deux d'un coup
     * (il faut qu'une longueur corrompue en ait absorbé deux entières), seule la première est rendue.
     */
    bool feed(uint8_t byte, BsbFrame& out_frame);

    /** true tant qu'une trame est en cours de réception (0xDC reçu, trame pas encore complète). */
    bool in_frame() const { return state_ != State::WaitSof; }

    /** Trames complètes rejetées pour CRC faux (les essais de resynchronisation qui échouent ne comptent pas). */
    uint32_t crc_error_count() const { return crc_error_count_; }

    /** Trames valides retrouvées dans les octets d'une trame rejetée. */
    uint32_t recovered_count() const { return recovered_count_; }

    /**
     * @brief Dernière trame rejetée pour CRC faux, telle qu'elle a été reçue (diagnostic) : `raw`, `raw_len`,
     *        adresses et commande sont renseignés, `crc` porte le CRC lu.
     */
    const BsbFrame& last_rejected() const { return rejected_; }

    /** CRC calculé sur les octets de la dernière trame rejetée (à comparer à `last_rejected().crc`). */
    uint16_t last_rejected_computed_crc() const { return rejected_computed_crc_; }

private:
    /** Issue du traitement d'un octet par l'automate. */
    enum class Step : uint8_t {
        NeedMore,  // Trame en cours, ou octet hors trame ignoré
        Frame,     // Trame complète et valide rendue
        BadHeader, // Longueur impossible : les octets absorbés sont dans working_
        BadCrc     // Trame complète au CRC faux : ses octets sont dans working_
    };

    Step step(uint8_t byte, BsbFrame& out_frame);
    bool rescan_rejected(BsbFrame& out_frame);

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
    // La trame en cours a commencé dans des octets relus après un rejet : si elle est rejetée à son
    // tour, c'est un essai de resynchronisation qui échoue, pas une nouvelle erreur sur le bus.
    bool     candidate_from_rescan_ = false;

    uint32_t crc_error_count_       = 0;
    uint32_t recovered_count_       = 0;
    BsbFrame rejected_              = {};
    uint16_t rejected_computed_crc_ = 0;
};

} // namespace bsb

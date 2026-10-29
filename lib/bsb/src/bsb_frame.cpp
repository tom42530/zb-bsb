#include "bsb_frame.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace bsb {

uint16_t BsbFrame::compute_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void BsbFrame::serialize() {
    if (payload_len > MAX_PAYLOAD_LEN) {
        payload_len = MAX_PAYLOAD_LEN;
    }

    const uint8_t total_len = static_cast<uint8_t>(HEADER_NO_PL + payload_len);
    size_t idx = 0;

    raw[idx++] = SOF_BYTE;
    raw[idx++] = static_cast<uint8_t>(src_addr | 0x80);
    raw[idx++] = dst_addr;
    raw[idx++] = total_len;
    raw[idx++] = static_cast<uint8_t>(command);

    // Particularité protocole BSB : pour GET, SET et INF, les 2 premiers octets
    // du field_id doivent être permutés lors de l'émission sur le bus.
    if (command == Command::Get || command == Command::Set || command == Command::Inf) {
        raw[idx++] = static_cast<uint8_t>((field_id >> 16) & 0xFF);
        raw[idx++] = static_cast<uint8_t>((field_id >> 24) & 0xFF);
    } else {
        raw[idx++] = static_cast<uint8_t>((field_id >> 24) & 0xFF);
        raw[idx++] = static_cast<uint8_t>((field_id >> 16) & 0xFF);
    }
    raw[idx++] = static_cast<uint8_t>((field_id >> 8) & 0xFF);
    raw[idx++] = static_cast<uint8_t>(field_id & 0xFF);

    for (uint8_t i = 0; i < payload_len; ++i) {
        raw[idx++] = payload[i];
    }

    crc = compute_crc16(raw, idx);
    raw[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    raw[idx++] = static_cast<uint8_t>(crc & 0xFF);

    raw_len = static_cast<uint8_t>(idx);
}

BsbFrame BsbFrame::make_get(uint8_t src, uint8_t dst, uint32_t field_id) {
    BsbFrame f;
    f.src_addr    = src;
    f.dst_addr    = dst;
    f.command     = Command::Get;
    f.field_id    = field_id;
    f.payload_len = 0;
    f.serialize();
    return f;
}

BsbFrame BsbFrame::make_set_uint8(uint8_t src, uint8_t dst, uint32_t field_id, uint8_t value, uint8_t enable_byte) {
    BsbFrame f;
    f.src_addr    = src;
    f.dst_addr    = dst;
    f.command     = Command::Set;
    f.field_id    = field_id;
    f.payload[0]  = (enable_byte == 0x06 && value == 0) ? 0x05 : enable_byte;
    f.payload[1]  = value;
    f.payload_len = 2;
    f.serialize();
    return f;
}

BsbFrame BsbFrame::make_set_temperature(uint8_t src, uint8_t dst, uint32_t field_id, float temp_c, uint8_t enable_byte) {
    BsbFrame f;
    f.src_addr    = src;
    f.dst_addr    = dst;
    f.command     = Command::Set;
    f.field_id    = field_id;

    const int16_t encoded = static_cast<int16_t>(std::lround(temp_c * 64.0f));
    f.payload[0]  = enable_byte;
    f.payload[1]  = static_cast<uint8_t>((encoded >> 8) & 0xFF);
    f.payload[2]  = static_cast<uint8_t>(encoded & 0xFF);
    f.payload_len = 3;
    f.serialize();
    return f;
}

BsbFrame BsbFrame::make_inf_room_temperature(uint8_t src, uint32_t field_id, float temp_c) {
    BsbFrame f;
    f.src_addr    = src;
    f.dst_addr    = 0x7F; // Broadcast
    f.command     = Command::Inf;
    f.field_id    = field_id;

    const int16_t encoded = static_cast<int16_t>(std::lround(temp_c * 64.0f));
    f.payload[0]  = static_cast<uint8_t>((encoded >> 8) & 0xFF);
    f.payload[1]  = static_cast<uint8_t>(encoded & 0xFF);
    f.payload[2]  = 0x00;
    f.payload_len = 3;
    f.serialize();
    return f;
}

DecodedValue BsbFrame::decode(ValueType expected_type) const {
    DecodedValue out;
    out.field_id = field_id;
    out.type     = (expected_type != ValueType::Unknown) ? expected_type : get_field_value_type(field_id);

    if (command != Command::Ret && command != Command::Inf) {
        out.valid = false;
        return out;
    }

    if (payload_len == 0) {
        out.valid = false;
        return out;
    }

    switch (out.type) {
        case ValueType::String: {
            size_t copy_len = 0;
            for (uint8_t i = 0; i < payload_len && copy_len < sizeof(out.str_val) - 1; ++i) {
                char c = static_cast<char>(payload[i]);
                if (c == '\0') {
                    break;
                }
                if (c >= 0x20 && c <= 0x7E) {
                    out.str_val[copy_len++] = c;
                }
            }
            out.str_val[copy_len] = '\0';
            out.valid = (copy_len > 0);
            break;
        }

        case ValueType::Binary: {
            if (payload_len >= 2) {
                out.flag_byte = payload[0];
                out.int_val   = payload[1];
                out.bool_val  = (payload[1] != 0);
                out.float_val = out.bool_val ? 1.0f : 0.0f;
                out.valid     = true;
            }
            break;
        }

        case ValueType::UInt8: {
            if (payload_len == 2) {
                out.flag_byte = payload[0];
                out.int_val   = payload[1];
                out.float_val = static_cast<float>(out.int_val);
                out.bool_val  = (out.int_val != 0);
                // Pour les pourcentages de modulation (8413, 8414), flag_byte != 0 signifie "---" (0%)
                if ((field_id == fields::COMPRESSOR_MOD || field_id == fields::BACKUP_MOD) && out.flag_byte != 0) {
                    out.int_val   = 0;
                    out.float_val = 0.0f;
                }
                out.valid = true;
            } else if (payload_len == 3) {
                // Certains codes statut BSB peuvent être retournés sur 16 bits [flag, msb, lsb]
                out.flag_byte = payload[0];
                out.int_val   = static_cast<uint16_t>((payload[1] << 8) | payload[2]);
                out.float_val = static_cast<float>(out.int_val);
                out.valid     = true;
            }
            break;
        }

        case ValueType::Int8: {
            if (payload_len == 2) {
                out.flag_byte = payload[0];
                out.int_val   = static_cast<int8_t>(payload[1]);
                out.float_val = static_cast<float>(out.int_val);
                out.valid     = (out.flag_byte == 0x00);
            }
            break;
        }

        case ValueType::Int16: {
            if (payload_len == 3) {
                out.flag_byte = payload[0];
                int16_t v     = static_cast<int16_t>((static_cast<uint16_t>(payload[1]) << 8) | payload[2]);
                out.int_val   = v;
                out.float_val = static_cast<float>(v);
                out.valid     = true;
            } else if (payload_len == 2) {
                out.flag_byte = payload[0];
                out.int_val   = payload[1];
                out.float_val = static_cast<float>(out.int_val);
                out.valid     = true;
            }
            break;
        }

        case ValueType::Int32: {
            if (payload_len == 5) {
                out.flag_byte = payload[0];
                int32_t v = (static_cast<int32_t>(payload[1]) << 24) |
                            (static_cast<int32_t>(payload[2]) << 16) |
                            (static_cast<int32_t>(payload[3]) << 8)  |
                            static_cast<int32_t>(payload[4]);
                out.int_val   = v;
                out.float_val = static_cast<float>(v);
                out.valid     = (out.flag_byte == 0x00);
            }
            break;
        }

        case ValueType::Temperature: {
            if (payload_len == 3) {
                out.flag_byte = payload[0];
                int16_t raw_t = static_cast<int16_t>((static_cast<uint16_t>(payload[1]) << 8) | payload[2]);
                out.int_val   = raw_t;
                out.float_val = static_cast<float>(raw_t) / 64.0f;
                // Sur BSB, flag_byte == 0x00 indique une mesure valide ;
                // flag_byte != 0 (ex: 0x01 / 0x05) indique une sonde absente ou "---"
                out.valid     = (out.flag_byte == 0x00) && (out.float_val > -60.0f && out.float_val < 150.0f);
            }
            break;
        }

        default:
            out.valid = false;
            break;
    }

    return out;
}

void BsbFrame::format_debug(char* out_buf, size_t out_size) const {
    if (!out_buf || out_size == 0) return;

    const char* cmd_str = "UNK";
    switch (command) {
        case Command::Inf:  cmd_str = "INF"; break;
        case Command::Set:  cmd_str = "SET"; break;
        case Command::Ack:  cmd_str = "ACK"; break;
        case Command::Nack: cmd_str = "NAK"; break;
        case Command::Get:  cmd_str = "GET"; break;
        case Command::Ret:  cmd_str = "RET"; break;
        default: break;
    }

    int written = snprintf(out_buf, out_size, "[%s] %02X->%02X field=0x%08lX len=%u pl=",
                           cmd_str, src_addr, dst_addr,
                           static_cast<unsigned long>(field_id),
                           static_cast<unsigned>(raw_len));
    if (written < 0 || static_cast<size_t>(written) >= out_size) return;

    size_t pos = static_cast<size_t>(written);
    for (uint8_t i = 0; i < payload_len && pos + 3 < out_size; ++i) {
        int n = snprintf(out_buf + pos, out_size - pos, "%02X", payload[i]);
        if (n > 0) pos += static_cast<size_t>(n);
    }
}

void BsbFrameParser::reset() {
    state_        = State::WaitSof;
    expected_len_ = 0;
    payload_idx_  = 0;
    working_.raw_len     = 0;
    working_.payload_len = 0;
}

bool BsbFrameParser::feed(uint8_t byte, BsbFrame& out_frame) {
    switch (state_) {
        case State::WaitSof:
            if (byte == BsbFrame::SOF_BYTE) {
                working_             = BsbFrame{};
                working_.raw[0]      = byte;
                working_.raw_len     = 1;
                state_               = State::SrcAddr;
            }
            break;

        case State::SrcAddr:
            if (byte == BsbFrame::SOF_BYTE) {
                // Resynchronisation immédiate si deux 0xDC se suivent
                working_         = BsbFrame{};
                working_.raw[0]  = byte;
                working_.raw_len = 1;
                state_           = State::SrcAddr;
            } else if ((byte & 0x80) != 0) {
                working_.raw[working_.raw_len++] = byte;
                working_.src_addr                = static_cast<uint8_t>(byte & 0x7F);
                state_                           = State::DstAddr;
            } else {
                reset();
            }
            break;

        case State::DstAddr:
            working_.raw[working_.raw_len++] = byte;
            working_.dst_addr                = byte;
            state_                           = State::Length;
            break;

        case State::Length:
            // Validation stricte de la taille pour éviter tout blocage sur octet bruité
            if (byte < BsbFrame::HEADER_NO_PL || byte > BsbFrame::MAX_FRAME_LEN) {
                reset();
                if (byte == BsbFrame::SOF_BYTE) {
                    working_.raw[0]  = byte;
                    working_.raw_len = 1;
                    state_           = State::SrcAddr;
                }
            } else {
                working_.raw[working_.raw_len++] = byte;
                expected_len_                    = byte;
                state_                           = State::CommandByte;
            }
            break;

        case State::CommandByte:
            working_.raw[working_.raw_len++] = byte;
            working_.command                 = static_cast<Command>(byte);
            state_                           = State::Field0;
            break;

        case State::Field0:
            working_.raw[working_.raw_len++] = byte;
            working_.field_id                = static_cast<uint32_t>(byte) << 24;
            state_                           = State::Field1;
            break;

        case State::Field1:
            working_.raw[working_.raw_len++] = byte;
            working_.field_id               |= static_cast<uint32_t>(byte) << 16;
            state_                           = State::Field2;
            break;

        case State::Field2:
            working_.raw[working_.raw_len++] = byte;
            working_.field_id               |= static_cast<uint32_t>(byte) << 8;
            state_                           = State::Field3;
            break;

        case State::Field3: {
            working_.raw[working_.raw_len++] = byte;
            working_.field_id               |= static_cast<uint32_t>(byte);

            // Si c'est une trame GET ou SET émise par un autre maître (ex: 0x0A),
            // remettons les 2 premiers octets dans l'ordre canonique.
            if (working_.command == Command::Get || working_.command == Command::Set) {
                working_.field_id = BsbFrame::swap_field_msb(working_.field_id);
            }

            const uint8_t pl_len = static_cast<uint8_t>(expected_len_ - BsbFrame::HEADER_NO_PL);
            working_.payload_len = pl_len;
            payload_idx_         = 0;

            if (pl_len > 0) {
                state_ = State::Payload;
            } else {
                state_ = State::CrcMsb;
            }
            break;
        }

        case State::Payload:
            working_.raw[working_.raw_len++] = byte;
            working_.payload[payload_idx_++] = byte;
            if (payload_idx_ >= working_.payload_len) {
                state_ = State::CrcMsb;
            }
            break;

        case State::CrcMsb:
            working_.raw[working_.raw_len++] = byte;
            working_.crc                     = static_cast<uint16_t>(byte) << 8;
            state_                           = State::CrcLsb;
            break;

        case State::CrcLsb: {
            working_.raw[working_.raw_len++] = byte;
            working_.crc                    |= static_cast<uint16_t>(byte);

            const uint16_t calc_crc = BsbFrame::compute_crc16(working_.raw, working_.raw_len - 2);
            const bool crc_ok = (calc_crc == working_.crc);
            if (crc_ok) {
                out_frame = working_;
            }
            reset();
            return crc_ok;
        }
    }

    return false;
}

} // namespace bsb

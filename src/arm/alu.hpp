#pragma once

#include <stdint.h>

namespace arm {

enum SRType { SRType_LSL, SRType_LSR, SRType_ASR, SRType_ROR, SRType_RRX };

struct ShiftResult { uint32_t result; bool carry; };
struct AddResult { uint32_t result; bool carry; bool overflow; };

inline ShiftResult LSL_C(uint32_t x, int shift) {
    uint32_t result = shift >= 32 ? 0u : (x << shift);
    bool carry = shift <= 32 ? ((x >> (32 - shift)) & 1u) != 0 : false;
    return { result, carry };
}

inline ShiftResult LSR_C(uint32_t x, int shift) {
    uint32_t result = shift >= 32 ? 0u : (x >> shift);
    bool carry = shift <= 32 ? ((x >> (shift - 1)) & 1u) != 0 : false;
    return { result, carry };
}

inline ShiftResult ASR_C(uint32_t x, int shift) {
    if (shift >= 32) {
        bool sign = (x >> 31) & 1u;
        return { sign ? 0xFFFFFFFFu : 0u, sign };
    }
    uint32_t result = (uint32_t)((int32_t)x >> shift);
    bool carry = ((x >> (shift - 1)) & 1u) != 0;
    return { result, carry };
}

inline ShiftResult ROR_C(uint32_t x, int shift) {
    int m = shift & 31;
    uint32_t result = m == 0 ? x : ((x >> m) | (x << (32 - m)));
    bool carry = ((result >> 31) & 1u) != 0;
    return { result, carry };
}

inline ShiftResult RRX_C(uint32_t x, bool carry_in) {
    uint32_t result = ((uint32_t)carry_in << 31) | (x >> 1);
    bool carry = (x & 1u) != 0;
    return { result, carry };
}

inline ShiftResult Shift_C(uint32_t value, SRType type, int amount, bool carry_in) {
    if (amount == 0) return { value, carry_in };
    switch (type) {
        case SRType_LSL: return LSL_C(value, amount);
        case SRType_LSR: return LSR_C(value, amount);
        case SRType_ASR: return ASR_C(value, amount);
        case SRType_ROR: return ROR_C(value, amount);
        case SRType_RRX: return RRX_C(value, carry_in);
    }
    return { value, carry_in };
}

inline uint32_t Shift(uint32_t value, SRType type, int amount, bool carry_in) {
    return Shift_C(value, type, amount, carry_in).result;
}

struct DecodedShift { SRType type; int amount; };

inline DecodedShift DecodeImmShift(uint32_t type, uint32_t imm5) {
    switch (type) {
        case 0: return { SRType_LSL, (int)imm5 };
        case 1: return { SRType_LSR, imm5 == 0 ? 32 : (int)imm5 };
        case 2: return { SRType_ASR, imm5 == 0 ? 32 : (int)imm5 };
        default:
            if (imm5 == 0) return { SRType_RRX, 1 };
            return { SRType_ROR, (int)imm5 };
    }
}

inline AddResult AddWithCarry(uint32_t x, uint32_t y, bool carry_in) {
    uint64_t usum = (uint64_t)x + (uint64_t)y + (uint64_t)carry_in;
    int64_t ssum = (int64_t)(int32_t)x + (int64_t)(int32_t)y + (int64_t)carry_in;
    uint32_t result = (uint32_t)usum;
    bool carry = (uint64_t)result != usum;
    bool overflow = (int64_t)(int32_t)result != ssum;
    return { result, carry, overflow };
}

inline ShiftResult ThumbExpandImm_C(uint32_t imm12, bool carry_in) {
    if (((imm12 >> 10) & 3u) == 0) {
        uint32_t imm8 = imm12 & 0xFFu;
        switch ((imm12 >> 8) & 3u) {
            case 0: return { imm8, carry_in };
            case 1: return { (imm8 << 16) | imm8, carry_in };
            case 2: return { (imm8 << 24) | (imm8 << 8), carry_in };
            default: return { (imm8 << 24) | (imm8 << 16) | (imm8 << 8) | imm8, carry_in };
        }
    }
    uint32_t unrotated = 0x80u | (imm12 & 0x7Fu);
    return ROR_C(unrotated, (int)((imm12 >> 7) & 0x1Fu));
}

inline uint32_t SignExtend(uint32_t x, int bits) {
    uint32_t m = 1u << (bits - 1);
    return (x ^ m) - m;
}

inline int BitCount(uint32_t x) { return __builtin_popcount(x); }

}

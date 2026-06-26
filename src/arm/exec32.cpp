#include "cpu.hpp"

namespace arm {

void exec_vfp(CPU& c, uint32_t hw1, uint32_t hw2);

static inline uint32_t bits(uint32_t v, int hi, int lo) {
    return (v >> lo) & ((1u << (hi - lo + 1)) - 1);
}
static inline uint32_t bit(uint32_t v, int n) { return (v >> n) & 1u; }

static void wnz(CPU& c, uint32_t r) {
    c.setFlag(FLAG_N, r & 0x80000000u);
    c.setFlag(FLAG_Z, r == 0);
}

static void dp_modified_immediate(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op = bits(hw1, 8, 5);
    uint32_t n = bits(hw1, 3, 0);
    uint32_t d = bits(hw2, 11, 8);
    bool S = bit(hw1, 4);
    uint32_t imm12 = (bit(hw1, 10) << 11) | (bits(hw2, 14, 12) << 8) | bits(hw2, 7, 0);
    ShiftResult e = ThumbExpandImm_C(imm12, c.C());
    uint32_t imm = e.result, rn = c.R[n];
    auto logic = [&](uint32_t r, bool test) {
        if (!test) c.R[d] = r;
        if (S) { wnz(c, r); c.setFlag(FLAG_C, e.carry); }
    };
    auto arith = [&](AddResult r, bool test) {
        if (!test) c.R[d] = r.result;
        if (S) { wnz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
    };
    switch (op) {
        case 0x0: logic(rn & imm, n == 15 ? false : (d == 15 && S)); if (d == 15 && S) { wnz(c, rn & imm); c.setFlag(FLAG_C, e.carry); } break;
        case 0x1: logic(rn & ~imm, false); break;
        case 0x2: logic(n == 15 ? imm : (rn | imm), false); break;
        case 0x3: logic(n == 15 ? ~imm : (rn | ~imm), false); break;
        case 0x4: if (d == 15 && S) { wnz(c, rn ^ imm); c.setFlag(FLAG_C, e.carry); } else logic(rn ^ imm, false); break;
        case 0x8: arith(AddWithCarry(rn, imm, false), d == 15 && S); break;
        case 0xA: arith(AddWithCarry(rn, imm, c.C()), false); break;
        case 0xB: arith(AddWithCarry(rn, ~imm, c.C()), false); break;
        case 0xD: arith(AddWithCarry(rn, ~imm, true), d == 15 && S); break;
        case 0xE: arith(AddWithCarry(~rn, imm, true), false); break;
        default: c.setFault("undef-dpmodimm"); break;
    }
}

static int32_t signed_sat(int64_t v, int bits_n, bool& sat) {
    int64_t hi = (1ll << (bits_n - 1)) - 1, lo = -(1ll << (bits_n - 1));
    sat = v > hi || v < lo;
    return (int32_t)(v > hi ? hi : v < lo ? lo : v);
}
static uint32_t unsigned_sat(int64_t v, int bits_n, bool& sat) {
    int64_t hi = (1ll << bits_n) - 1;
    sat = v > hi || v < 0;
    return (uint32_t)(v > hi ? hi : v < 0 ? 0 : v);
}

static void dp_plain_immediate(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op = bits(hw1, 8, 4);
    uint32_t n = bits(hw1, 3, 0);
    uint32_t d = bits(hw2, 11, 8);
    uint32_t i = bit(hw1, 10), imm3 = bits(hw2, 14, 12), imm8 = bits(hw2, 7, 0);
    uint32_t imm12 = (i << 11) | (imm3 << 8) | imm8;
    switch (op) {
        case 0x00: c.R[d] = (n == 15 ? (c.pc_read & ~3u) : c.R[n]) + imm12; break;
        case 0x0A: c.R[d] = (n == 15 ? (c.pc_read & ~3u) : c.R[n]) - imm12; break;
        case 0x04: c.R[d] = (bits(hw1, 3, 0) << 12) | imm12; break;
        case 0x0C: c.R[d] = (c.R[d] & 0xFFFF) | (((bits(hw1, 3, 0) << 12) | imm12) << 16); break;
        case 0x14: case 0x1C: {
            uint32_t lsb = (imm3 << 2) | bits(hw2, 7, 6);
            uint32_t width = bits(hw2, 4, 0) + 1;
            uint32_t mask = width >= 32 ? 0xFFFFFFFFu : ((1u << width) - 1);
            uint32_t v = (c.R[n] >> lsb) & mask;
            if (op == 0x14 && (v & (1u << (width - 1)))) v |= ~mask;
            c.R[d] = v;
            break;
        }
        case 0x16: {
            uint32_t lsb = (imm3 << 2) | bits(hw2, 7, 6);
            uint32_t msb = bits(hw2, 4, 0);
            if (msb < lsb) { c.setFault("bfi-unpred"); break; }
            uint32_t width = msb - lsb + 1;
            uint32_t mask = (width >= 32 ? 0xFFFFFFFFu : ((1u << width) - 1)) << lsb;
            uint32_t src = n == 15 ? 0 : c.R[n];
            c.R[d] = (c.R[d] & ~mask) | ((src << lsb) & mask);
            break;
        }
        case 0x10: case 0x12: case 0x18: case 0x1A: {
            uint32_t sh = bit(hw1, 5);
            uint32_t amt = (imm3 << 2) | bits(hw2, 7, 6);
            DecodedShift ds = DecodeImmShift((sh << 1), amt);
            int32_t operand = (int32_t)Shift(c.R[n], ds.type, ds.amount, c.C());
            bool sat;
            if (op >= 0x18) c.R[d] = unsigned_sat(operand, bits(hw2, 4, 0), sat);
            else c.R[d] = (uint32_t)signed_sat(operand, bits(hw2, 4, 0) + 1, sat);
            if (sat) c.setFlag(FLAG_Q, true);
            break;
        }
        default: c.setFault("undef-dpplain"); break;
    }
}

static void branches_and_misc(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op12_14 = bits(hw2, 14, 12);
    if ((op12_14 & 0x5) == 0) {
        uint32_t cond = bits(hw1, 9, 6);
        if (cond < 0xE) {
            uint32_t S = bit(hw1, 10), J1 = bit(hw2, 13), J2 = bit(hw2, 11);
            uint32_t imm6 = bits(hw1, 5, 0), imm11 = bits(hw2, 10, 0);
            uint32_t imm = (S << 20) | (J2 << 19) | (J1 << 18) | (imm6 << 12) | (imm11 << 1);
            if (c.ConditionPassed(cond)) c.branchWritePC(c.pc_read + SignExtend(imm, 21));
            return;
        }
        uint32_t sysop = bits(hw1, 9, 4);
        if (sysop == 0x38 || sysop == 0x39) {
            uint32_t rn = bits(hw1, 3, 0), sysm = bits(hw2, 7, 0), v = c.R[rn];
            if (sysm == 8) c.sp_main = v & ~3u;
            else if (sysm == 9) c.sp_process = v & ~3u;
            else if (sysm <= 3) c.xpsr = (c.xpsr & ~0xF8000000u) | (v & 0xF8000000u);
            else if (sysm == 16) c.primask = v & 1u;
            else if (sysm == 17) c.basepri = v & 0xFFu;
            else if (sysm == 18) c.basepri = v & 0xFFu;
            else if (sysm == 19) c.faultmask = v & 1u;
            else if (sysm == 20) c.control = v & 0x7u;
            return;
        }
        if (sysop == 0x3B) return;
        if (sysop == 0x3E || sysop == 0x3F) {
            uint32_t rd = bits(hw2, 11, 8), sysm = bits(hw2, 7, 0), v = 0;
            if (sysm <= 3) v = c.xpsr & 0xF800FDFFu;
            else if (sysm >= 5 && sysm <= 7) v = c.xpsr & 0x0000FDFFu;
            else if (sysm == 8) v = c.sp_main;
            else if (sysm == 9) v = c.sp_process;
            else if (sysm == 16) v = c.primask;
            else if (sysm == 17 || sysm == 18) v = c.basepri;
            else if (sysm == 19) v = c.faultmask;
            else if (sysm == 20) v = c.control;
            c.R[rd] = v;
            return;
        }
        c.setFault("undef-sysctl");
        return;
    }
    uint32_t S = bit(hw1, 10), imm10 = bits(hw1, 9, 0);
    uint32_t J1 = bit(hw2, 13), J2 = bit(hw2, 11), imm11 = bits(hw2, 10, 0);
    uint32_t I1 = (~(J1 ^ S)) & 1u, I2 = (~(J2 ^ S)) & 1u;
    uint32_t imm = (S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1);
    bool link = bit(hw2, 14);
    if (link) c.R[14] = c.pc_read | 1u;
    c.branchWritePC(c.pc_read + SignExtend(imm, 25));
}

static void load_store_multiple(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op = bits(hw1, 8, 7);
    bool L = bit(hw1, 4), W = bit(hw1, 5);
    uint32_t n = bits(hw1, 3, 0);
    uint32_t regs = hw2 & 0xDFFFu;
    int count = BitCount(regs);
    bool db = (op == 2);
    uint32_t base = c.R[n];
    uint32_t addr = db ? base - 4 * count : base;
    for (int i = 0; i < 16; ++i) if (regs & (1u << i)) {
        if (L) { if (i == 15) c.bxWritePC(c.mem->read32(addr)); else c.R[i] = c.mem->read32(addr); }
        else c.mem->write32(addr, c.R[i]);
        addr += 4;
    }
    if (W) c.R[n] = db ? base - 4 * count : base + 4 * count;
}

static void load_store_dual_excl_tb(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op1 = bits(hw1, 8, 7), op2 = bits(hw1, 5, 4), op3 = bits(hw2, 7, 4);
    uint32_t n = bits(hw1, 3, 0);
    if (op1 == 0 && op2 == 1 && (hw1 & 0x0010)) {
        uint32_t t = bits(hw2, 15, 12), imm8 = bits(hw2, 7, 0) << 2;
        c.R[t] = c.mem->read32(c.R[n] + imm8);
        return;
    }
    if (op1 == 0 && op2 == 0) {
        uint32_t d = bits(hw2, 11, 8), t = bits(hw2, 15, 12), imm8 = bits(hw2, 7, 0) << 2;
        c.mem->write32(c.R[n] + imm8, c.R[t]);
        c.R[d] = 0;
        return;
    }
    if (op1 == 1 && op2 == 1 && op3 <= 1) {
        uint32_t rm = bits(hw2, 3, 0);
        bool half = bit(hw2, 4);
        uint32_t base = n == 15 ? c.pc_read - 4 : c.R[n];
        uint32_t off = half ? c.mem->read16(base + (c.R[rm] << 1)) : c.mem->read8(base + c.R[rm]);
        c.branchWritePC(c.pc_read + 2 * off);
        return;
    }
    if (op1 == 1 && op2 == 1) {
        uint32_t t = bits(hw2, 15, 12), t2 = bits(hw2, 11, 8), d = bits(hw2, 3, 0);
        uint32_t addr = c.R[n];
        switch (op3) {
            case 0x4: c.R[t] = c.mem->read8(addr); return;
            case 0x5: c.R[t] = c.mem->read16(addr); return;
            case 0x7: c.R[t] = c.mem->read32(addr); c.R[t2] = c.mem->read32(addr + 4); return;
            default: c.setFault("undef-ldrex"); return;
        }
        (void)d;
    }
    if (op1 == 1 && op2 == 0) {
        uint32_t t = bits(hw2, 15, 12), t2 = bits(hw2, 11, 8), d = bits(hw2, 3, 0);
        uint32_t addr = c.R[n];
        switch (op3) {
            case 0x4: c.mem->write8(addr, c.R[t]); c.R[d] = 0; return;
            case 0x5: c.mem->write16(addr, c.R[t]); c.R[d] = 0; return;
            case 0x7: c.mem->write32(addr, c.R[t]); c.mem->write32(addr + 4, c.R[t2]); c.R[d] = 0; return;
            default: c.setFault("undef-strex"); return;
        }
    }
    bool P = bit(hw1, 8), U = bit(hw1, 7), W = bit(hw1, 5), L = bit(hw1, 4);
    uint32_t t = bits(hw2, 15, 12), t2 = bits(hw2, 11, 8), imm8 = bits(hw2, 7, 0) << 2;
    uint32_t base = c.R[n];
    uint32_t off = U ? imm8 : (uint32_t)-(int32_t)imm8;
    uint32_t addr = P ? base + off : base;
    if (L) { c.R[t] = c.mem->read32(addr); c.R[t2] = c.mem->read32(addr + 4); }
    else { c.mem->write32(addr, c.R[t]); c.mem->write32(addr + 4, c.R[t2]); }
    if (W) c.R[n] = base + off;
}

static void dp_shifted_register(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op = bits(hw1, 8, 5);
    bool S = bit(hw1, 4);
    uint32_t n = bits(hw1, 3, 0), d = bits(hw2, 11, 8), m = bits(hw2, 3, 0);
    uint32_t imm5 = (bits(hw2, 14, 12) << 2) | bits(hw2, 7, 6);
    DecodedShift ds = DecodeImmShift(bits(hw2, 5, 4), imm5);
    ShiftResult sh = Shift_C(c.R[m], ds.type, ds.amount, c.C());
    uint32_t rn = c.R[n], b = sh.result;
    auto logic = [&](uint32_t r, bool test) {
        if (!test) c.R[d] = r;
        if (S) { wnz(c, r); c.setFlag(FLAG_C, sh.carry); }
    };
    auto arith = [&](AddResult r, bool test) {
        if (!test) c.R[d] = r.result;
        if (S) { wnz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
    };
    switch (op) {
        case 0x0: logic(rn & b, d == 15 && S); break;
        case 0x1: logic(rn & ~b, false); break;
        case 0x2: logic(n == 15 ? b : (rn | b), false); break;
        case 0x3: logic(n == 15 ? ~b : (rn | ~b), false); break;
        case 0x4: logic(rn ^ b, d == 15 && S); break;
        case 0x8: arith(AddWithCarry(rn, b, false), d == 15 && S); break;
        case 0xA: arith(AddWithCarry(rn, b, c.C()), false); break;
        case 0xB: arith(AddWithCarry(rn, ~b, c.C()), false); break;
        case 0xD: arith(AddWithCarry(rn, ~b, true), d == 15 && S); break;
        case 0xE: arith(AddWithCarry(~rn, b, true), false); break;
        default: c.setFault("undef-dpshreg"); break;
    }
}

static void load_store_single(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t size = bits(hw1, 6, 5);
    bool load = bit(hw1, 4);
    bool sign = bit(hw1, 8);
    uint32_t n = bits(hw1, 3, 0), t = bits(hw2, 15, 12);
    uint32_t addr;
    bool wback = false; uint32_t wval = 0;
    if (n == 15 && load) {
        bool u = bit(hw1, 7);
        uint32_t imm12 = bits(hw2, 11, 0);
        addr = (c.pc_read & ~3u) + (u ? imm12 : (uint32_t)-(int32_t)imm12);
    } else if (bit(hw1, 7)) {
        addr = c.R[n] + bits(hw2, 11, 0);
    } else if (bits(hw2, 11, 11) == 0) {
        uint32_t m = bits(hw2, 3, 0), sh = bits(hw2, 5, 4);
        addr = c.R[n] + (c.R[m] << sh);
    } else {
        bool P = bit(hw2, 10), U = bit(hw2, 9), W = bit(hw2, 8);
        uint32_t imm8 = bits(hw2, 7, 0);
        uint32_t base = c.R[n];
        uint32_t off = U ? imm8 : (uint32_t)-(int32_t)imm8;
        addr = P ? base + off : base;
        if (W || !P) { wback = true; wval = base + off; }
    }
    if (load) {
        uint32_t v;
        if (size == 0) { v = c.mem->read8(addr); if (sign) v = (uint32_t)(int32_t)(int8_t)v; }
        else if (size == 1) { v = c.mem->read16(addr); if (sign) v = (uint32_t)(int32_t)(int16_t)v; }
        else v = c.mem->read32(addr);
        if (wback) c.R[n] = wval;
        c.writeR(t, v);
    } else {
        if (size == 0) c.mem->write8(addr, c.R[t]);
        else if (size == 1) c.mem->write16(addr, c.R[t]);
        else c.mem->write32(addr, c.R[t]);
        if (wback) c.R[n] = wval;
    }
}

static int32_t sat32q(CPU& c, int64_t v) {
    if (v > 0x7FFFFFFFll) { c.setFlag(FLAG_Q, true); return 0x7FFFFFFF; }
    if (v < -0x80000000ll) { c.setFlag(FLAG_Q, true); return (int32_t)0x80000000; }
    return (int32_t)v;
}

static void dp_register(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t a = bits(hw1, 7, 4), b = bits(hw2, 7, 4);
    uint32_t n = bits(hw1, 3, 0), d = bits(hw2, 11, 8), m = bits(hw2, 3, 0);
    if (b == 0 && (a & 0x8) == 0 && (a & 0x1) == 0) {
        SRType t = (SRType)((a >> 1) & 3);
        ShiftResult s = Shift_C(c.R[n], t, c.R[m] & 0xFF, c.C());
        c.R[d] = s.result;
        if (bit(hw1, 4)) { wnz(c, s.result); c.setFlag(FLAG_C, s.carry); }
        return;
    }
    if ((b & 0x8) && a <= 0x5) {
        uint32_t rot = bits(hw2, 5, 4) * 8;
        uint32_t v = (c.R[m] >> rot) | (c.R[m] << (32 - rot));
        uint32_t ext;
        switch (a) {
            case 0: ext = (uint32_t)(int32_t)(int16_t)v; break;
            case 1: ext = v & 0xFFFF; break;
            case 4: ext = (uint32_t)(int32_t)(int8_t)v; break;
            case 5: ext = v & 0xFF; break;
            default: ext = v; break;
        }
        c.R[d] = n == 15 ? ext : (c.R[n] + ext);
        return;
    }
    if ((b & 0xC) == 0x8) {
        uint32_t v = c.R[m], sub = b & 0x3;
        if (a == 0x9) {
            switch (sub) {
                case 0: c.R[d] = __builtin_bswap32(v); return;
                case 1: c.R[d] = ((v & 0x00FF00FF) << 8) | ((v >> 8) & 0x00FF00FF); return;
                case 2: { uint32_t r = 0; for (int i = 0; i < 32; ++i) if (v & (1u << i)) r |= 1u << (31 - i); c.R[d] = r; return; }
                case 3: c.R[d] = (uint32_t)(int32_t)(int16_t)(((v & 0xFF) << 8) | ((v >> 8) & 0xFF)); return;
            }
        }
        if (a == 0xB) { c.R[d] = v ? (uint32_t)__builtin_clz(v) : 32u; return; }
        if (a == 0x8) {
            int64_t x = (int32_t)c.R[n], y = (int32_t)c.R[m], r;
            switch (sub) {
                case 0: r = y + x; break;
                case 1: r = y - x; break;
                case 2: r = (int64_t)y + sat32q(c, 2 * x); break;
                default: r = (int64_t)y - sat32q(c, 2 * x); break;
            }
            c.R[d] = (uint32_t)sat32q(c, r);
            return;
        }
    }
    c.setFault("undef-dpreg");
}

static void multiply(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op1 = bits(hw1, 6, 4), op2 = bits(hw2, 5, 4);
    uint32_t n = bits(hw1, 3, 0), m = bits(hw2, 3, 0);
    uint32_t a = bits(hw2, 15, 12), d = bits(hw2, 11, 8);
    int16_t nl = (int16_t)c.R[n], nh = (int16_t)(c.R[n] >> 16);
    int16_t ml = (int16_t)c.R[m], mh = (int16_t)(c.R[m] >> 16);
    switch (op1) {
        case 0: { uint32_t p = c.R[n] * c.R[m]; c.R[d] = (op2 & 1) ? (c.R[a] - p) : (a == 15 ? p : c.R[a] + p); return; }
        case 1: { int32_t x = (op2 & 1) ? nh : nl, y = (op2 & 2) ? mh : ml, p = x * y; c.R[d] = a == 15 ? (uint32_t)p : (uint32_t)((int32_t)c.R[a] + p); return; }
        case 3: { int32_t y = (op2 & 1) ? mh : ml; int64_t p = ((int64_t)(int32_t)c.R[n] * y) >> 16; c.R[d] = a == 15 ? (uint32_t)p : (uint32_t)((int32_t)c.R[a] + (int32_t)p); return; }
        case 2: { int32_t mlo = (op2 & 1) ? mh : ml, mhi = (op2 & 1) ? ml : mh, p = nl * mlo + nh * mhi; c.R[d] = a == 15 ? (uint32_t)p : (uint32_t)((int32_t)c.R[a] + p); return; }
        case 4: { int32_t mlo = (op2 & 1) ? mh : ml, mhi = (op2 & 1) ? ml : mh, p = nl * mlo - nh * mhi; c.R[d] = a == 15 ? (uint32_t)p : (uint32_t)((int32_t)c.R[a] + p); return; }
        case 5: { int64_t acc = a == 15 ? 0 : ((int64_t)(int32_t)c.R[a] << 32); if (op2 & 2) acc = -acc; int64_t p = acc + (int64_t)(int32_t)c.R[n] * (int32_t)c.R[m]; if (op2 & 1) p += 0x80000000ll; c.R[d] = (uint32_t)(p >> 32); return; }
        case 7: { uint32_t sad = 0; for (int i = 0; i < 4; ++i) { int dd = (int)((c.R[n] >> (i * 8)) & 0xFF) - (int)((c.R[m] >> (i * 8)) & 0xFF); sad += (uint32_t)(dd < 0 ? -dd : dd); } c.R[d] = a == 15 ? sad : c.R[a] + sad; return; }
        default: c.setFault("undef-mul"); return;
    }
}

static void long_multiply(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op1 = bits(hw1, 6, 4), op2 = bits(hw2, 7, 4);
    uint32_t n = bits(hw1, 3, 0), m = bits(hw2, 3, 0);
    uint32_t lo = bits(hw2, 15, 12), hi = bits(hw2, 11, 8);
    switch (op1) {
        case 0: { int64_t p = (int64_t)(int32_t)c.R[n] * (int32_t)c.R[m]; c.R[lo] = (uint32_t)p; c.R[hi] = (uint32_t)((uint64_t)p >> 32); return; }
        case 2: { uint64_t p = (uint64_t)c.R[n] * c.R[m]; c.R[lo] = (uint32_t)p; c.R[hi] = (uint32_t)(p >> 32); return; }
        case 4: { int64_t acc = (int64_t)(((uint64_t)c.R[hi] << 32) | c.R[lo]); acc += (int64_t)(int32_t)c.R[n] * (int32_t)c.R[m]; c.R[lo] = (uint32_t)acc; c.R[hi] = (uint32_t)((uint64_t)acc >> 32); return; }
        case 6: {
            if (op2 == 0x6) { uint64_t r = (uint64_t)c.R[n] * c.R[m] + c.R[hi] + c.R[lo]; c.R[lo] = (uint32_t)r; c.R[hi] = (uint32_t)(r >> 32); return; }
            uint64_t acc = ((uint64_t)c.R[hi] << 32) | c.R[lo]; acc += (uint64_t)c.R[n] * c.R[m]; c.R[lo] = (uint32_t)acc; c.R[hi] = (uint32_t)(acc >> 32); return;
        }
        case 1: { int32_t q = c.R[m] == 0 ? 0 : (int32_t)c.R[n] / (int32_t)c.R[m]; c.R[hi] = (uint32_t)q; return; }
        case 3: { uint32_t q = c.R[m] == 0 ? 0 : c.R[n] / c.R[m]; c.R[hi] = q; return; }
        default: c.setFault("undef-lmul"); return;
    }
}

void exec_thumb32(CPU& c, uint32_t hw1, uint32_t hw2) {
    uint32_t op1 = bits(hw1, 12, 11);
    if (op1 == 1) {
        uint32_t op2 = bits(hw1, 10, 4);
        if ((op2 & 0x64) == 0x00) { load_store_multiple(c, hw1, hw2); return; }
        if ((op2 & 0x64) == 0x04) { load_store_dual_excl_tb(c, hw1, hw2); return; }
        if ((op2 & 0x60) == 0x20) { dp_shifted_register(c, hw1, hw2); return; }
        exec_vfp(c, hw1, hw2);
        return;
    }
    if (op1 == 2) {
        if (bit(hw2, 15)) { branches_and_misc(c, hw1, hw2); return; }
        if (bit(hw1, 9)) dp_plain_immediate(c, hw1, hw2);
        else dp_modified_immediate(c, hw1, hw2);
        return;
    }
    uint32_t op2 = bits(hw1, 10, 4);
    if ((op2 & 0x71) == 0x00) { load_store_single(c, hw1, hw2); return; }
    if ((op2 & 0x67) == 0x01) { load_store_single(c, hw1, hw2); return; }
    if ((op2 & 0x67) == 0x03) { load_store_single(c, hw1, hw2); return; }
    if ((op2 & 0x67) == 0x05) { load_store_single(c, hw1, hw2); return; }
    if ((op2 & 0x70) == 0x20) { dp_register(c, hw1, hw2); return; }
    if ((op2 & 0x78) == 0x30) { multiply(c, hw1, hw2); return; }
    if ((op2 & 0x78) == 0x38) { long_multiply(c, hw1, hw2); return; }
    exec_vfp(c, hw1, hw2);
}

void exec_vfp(CPU& c, uint32_t hw1, uint32_t hw2) {
    (void)hw1; (void)hw2;
    c.setFault("vfp-unimpl");
}

}

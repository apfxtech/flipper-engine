#include "cpu.hpp"

namespace arm {

void exec_thumb32(CPU& c, uint32_t hw1, uint32_t hw2);

static inline uint32_t bits(uint32_t v, int hi, int lo) {
    return (v >> lo) & ((1u << (hi - lo + 1)) - 1);
}
static inline uint32_t bit(uint32_t v, int n) { return (v >> n) & 1u; }

void cpu_reset(CPU& c, uint32_t vtor) {
    for (int i = 0; i < 16; ++i) c.R[i] = 0;
    for (int i = 0; i < 32; ++i) c.S[i] = 0;
    c.fpscr = 0;
    c.xpsr = FLAG_T;
    c.primask = c.faultmask = c.basepri = c.control = 0;
    c.mode = MODE_THREAD;
    c.itstate = 0;
    c.fault = false;
    c.fault_msg = nullptr;
    c.cycles = 0;
    c.sp_main = c.mem->read32(vtor + 0) & ~3u;
    uint32_t reset = c.mem->read32(vtor + 4);
    c.R[13] = c.sp_main;
    c.R[15] = reset & ~1u;
    c.R[14] = 0xFFFFFFFFu;
}

static void wflags_nz(CPU& c, uint32_t r) {
    c.setFlag(FLAG_N, r & 0x80000000u);
    c.setFlag(FLAG_Z, r == 0);
}

static void shift_imm(CPU& c, uint32_t op, uint32_t type, uint32_t imm5) {
    uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3);
    DecodedShift ds = DecodeImmShift(type, imm5);
    ShiftResult s = Shift_C(c.R[m], ds.type, ds.amount, c.C());
    c.R[d] = s.result;
    if (!c.InITBlock()) { wflags_nz(c, s.result); c.setFlag(FLAG_C, s.carry); }
}

static void decode16_shift_addsub_movcmp(CPU& c, uint32_t op) {
    uint32_t opcode = bits(op, 13, 9);
    if (opcode < 0x0C) { shift_imm(c, op, bits(op, 12, 11), bits(op, 10, 6)); return; }
    bool setflags = !c.InITBlock();
    switch (opcode) {
        case 0x0C: case 0x0D: {
            uint32_t d = bits(op, 2, 0), n = bits(op, 5, 3), m = bits(op, 8, 6);
            AddResult r = opcode == 0x0C ? AddWithCarry(c.R[n], c.R[m], false)
                                         : AddWithCarry(c.R[n], ~c.R[m], true);
            c.R[d] = r.result;
            if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
            break;
        }
        case 0x0E: case 0x0F: {
            uint32_t d = bits(op, 2, 0), n = bits(op, 5, 3), imm = bits(op, 8, 6);
            AddResult r = opcode == 0x0E ? AddWithCarry(c.R[n], imm, false)
                                         : AddWithCarry(c.R[n], ~imm, true);
            c.R[d] = r.result;
            if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
            break;
        }
        case 0x10: case 0x11: case 0x12: case 0x13: {
            uint32_t d = bits(op, 10, 8), imm = bits(op, 7, 0);
            c.R[d] = imm;
            if (setflags) wflags_nz(c, imm);
            break;
        }
        case 0x14: case 0x15: case 0x16: case 0x17: {
            uint32_t n = bits(op, 10, 8), imm = bits(op, 7, 0);
            AddResult r = AddWithCarry(c.R[n], ~imm, true);
            wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow);
            break;
        }
        case 0x18: case 0x19: case 0x1A: case 0x1B: {
            uint32_t dn = bits(op, 10, 8), imm = bits(op, 7, 0);
            AddResult r = AddWithCarry(c.R[dn], imm, false);
            c.R[dn] = r.result;
            if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
            break;
        }
        default: {
            uint32_t dn = bits(op, 10, 8), imm = bits(op, 7, 0);
            AddResult r = AddWithCarry(c.R[dn], ~imm, true);
            c.R[dn] = r.result;
            if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); }
            break;
        }
    }
}

static void decode16_data_processing(CPU& c, uint32_t op) {
    uint32_t opcode = bits(op, 9, 6);
    uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3);
    uint32_t n = d;
    bool setflags = !c.InITBlock();
    uint32_t rn = c.R[n], rm = c.R[m];
    auto logic = [&](uint32_t r, bool carry) {
        c.R[d] = r;
        if (setflags) { wflags_nz(c, r); c.setFlag(FLAG_C, carry); }
    };
    switch (opcode) {
        case 0x0: { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); logic(rn & s.result, s.carry); break; }
        case 0x1: { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); logic(rn ^ s.result, s.carry); break; }
        case 0x2: { ShiftResult s = Shift_C(rn, SRType_LSL, rm & 0xFF, c.C()); logic(s.result, s.carry); break; }
        case 0x3: { ShiftResult s = Shift_C(rn, SRType_LSR, rm & 0xFF, c.C()); logic(s.result, s.carry); break; }
        case 0x4: { ShiftResult s = Shift_C(rn, SRType_ASR, rm & 0xFF, c.C()); logic(s.result, s.carry); break; }
        case 0x5: { AddResult r = AddWithCarry(rn, rm, c.C()); c.R[d] = r.result; if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); } break; }
        case 0x6: { AddResult r = AddWithCarry(rn, ~rm, c.C()); c.R[d] = r.result; if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); } break; }
        case 0x7: { ShiftResult s = Shift_C(rn, SRType_ROR, rm & 0xFF, c.C()); logic(s.result, s.carry); break; }
        case 0x8: { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); uint32_t r = rn & s.result; wflags_nz(c, r); c.setFlag(FLAG_C, s.carry); break; }
        case 0x9: { AddResult r = AddWithCarry(~rn, 0, true); c.R[d] = r.result; if (setflags) { wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); } break; }
        case 0xA: { AddResult r = AddWithCarry(rn, ~rm, true); wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); break; }
        case 0xB: { AddResult r = AddWithCarry(rn, rm, false); wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow); break; }
        case 0xC: { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); logic(rn | s.result, s.carry); break; }
        case 0xD: { uint32_t r = rn * rm; c.R[d] = r; if (setflags) wflags_nz(c, r); break; }
        case 0xE: { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); logic(rn & ~s.result, s.carry); break; }
        default:  { ShiftResult s = Shift_C(rm, SRType_LSL, 0, c.C()); logic(~s.result, s.carry); break; }
    }
}

static void decode16_special_data(CPU& c, uint32_t op) {
    uint32_t hi = bits(op, 9, 8);
    if (hi == 0) {
        uint32_t d = (bit(op, 7) << 3) | bits(op, 2, 0), m = bits(op, 6, 3);
        c.writeR(d, c.readR(d) + c.readR(m));
    } else if (hi == 1) {
        uint32_t n = (bit(op, 7) << 3) | bits(op, 2, 0), m = bits(op, 6, 3);
        AddResult r = AddWithCarry(c.readR(n), ~c.readR(m), true);
        wflags_nz(c, r.result); c.setFlag(FLAG_C, r.carry); c.setFlag(FLAG_V, r.overflow);
    } else if (hi == 2) {
        uint32_t d = (bit(op, 7) << 3) | bits(op, 2, 0), m = bits(op, 6, 3);
        c.writeR(d, c.readR(m));
    } else {
        uint32_t m = bits(op, 6, 3);
        if (bit(op, 7) == 0) {
            c.bxWritePC(c.readR(m));
        } else {
            c.R[14] = (c.pc_read - 2) | 1u;
            c.blxWritePC(c.readR(m));
        }
    }
}

static void load_store_single(CPU& c, uint32_t op) {
    uint32_t opA = bits(op, 15, 12);
    if (opA == 0x5) {
        uint32_t opB = bits(op, 11, 9);
        uint32_t m = bits(op, 8, 6), n = bits(op, 5, 3), t = bits(op, 2, 0);
        uint32_t addr = c.R[n] + c.R[m];
        switch (opB) {
            case 0: c.mem->write32(addr, c.R[t]); break;
            case 1: c.mem->write16(addr, c.R[t]); break;
            case 2: c.mem->write8(addr, c.R[t]); break;
            case 3: c.R[t] = (uint32_t)(int32_t)(int8_t)c.mem->read8(addr); break;
            case 4: c.R[t] = c.mem->read32(addr); break;
            case 5: c.R[t] = c.mem->read16(addr); break;
            case 6: c.R[t] = c.mem->read8(addr); break;
            default: c.R[t] = (uint32_t)(int32_t)(int16_t)c.mem->read16(addr); break;
        }
        return;
    }
    if (opA == 0x9) {
        bool load = bit(op, 11);
        uint32_t t = bits(op, 10, 8), imm = bits(op, 7, 0) << 2;
        uint32_t addr = c.R[13] + imm;
        if (load) c.R[t] = c.mem->read32(addr); else c.mem->write32(addr, c.R[t]);
        return;
    }
    bool load = bit(op, 11);
    uint32_t imm5 = bits(op, 10, 6), n = bits(op, 5, 3), t = bits(op, 2, 0);
    if (opA == 0x6) {
        uint32_t addr = c.R[n] + (imm5 << 2);
        if (load) c.R[t] = c.mem->read32(addr); else c.mem->write32(addr, c.R[t]);
    } else if (opA == 0x7) {
        uint32_t addr = c.R[n] + imm5;
        if (load) c.R[t] = c.mem->read8(addr); else c.mem->write8(addr, c.R[t]);
    } else {
        uint32_t addr = c.R[n] + (imm5 << 1);
        if (load) c.R[t] = c.mem->read16(addr); else c.mem->write16(addr, c.R[t]);
    }
}

static void decode16_misc(CPU& c, uint32_t op) {
    uint32_t o = bits(op, 11, 5);
    if (o == 0x33) { return; }
    if ((o & 0x7C) == 0x00) {
        uint32_t imm = bits(op, 6, 0) << 2;
        c.R[13] = c.R[13] + imm;
        return;
    }
    if ((o & 0x7C) == 0x04) {
        uint32_t imm = bits(op, 6, 0) << 2;
        c.R[13] = c.R[13] - imm;
        return;
    }
    if ((o & 0x78) == 0x08 || (o & 0x78) == 0x18 || (o & 0x78) == 0x48 || (o & 0x78) == 0x58) {
        bool nonzero = bit(op, 11);
        uint32_t imm = (bit(op, 9) << 6) | (bits(op, 7, 3) << 1);
        uint32_t n = bits(op, 2, 0);
        bool z = c.R[n] == 0;
        if (nonzero != z) c.branchWritePC(c.pc_read + imm);
        return;
    }
    if ((o & 0x7E) == 0x10) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); c.R[d] = (uint32_t)(int32_t)(int16_t)c.R[m]; return; }
    if ((o & 0x7E) == 0x12) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); c.R[d] = (uint32_t)(int32_t)(int8_t)c.R[m]; return; }
    if ((o & 0x7E) == 0x14) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); c.R[d] = c.R[m] & 0xFFFF; return; }
    if ((o & 0x7E) == 0x16) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); c.R[d] = c.R[m] & 0xFF; return; }
    if ((o & 0x70) == 0x20) {
        uint32_t regs = bits(op, 7, 0) | (bit(op, 8) << 14);
        uint32_t addr = c.R[13] - 4 * BitCount(regs);
        uint32_t a = addr;
        for (int i = 0; i < 15; ++i) if (regs & (1u << i)) { c.mem->write32(a, c.R[i]); a += 4; }
        c.R[13] = addr;
        return;
    }
    if ((o & 0x7E) == 0x50) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); c.R[d] = __builtin_bswap32(c.R[m]); return; }
    if ((o & 0x7E) == 0x52) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); uint32_t v = c.R[m]; c.R[d] = ((v & 0x00FF00FF) << 8) | ((v >> 8) & 0x00FF00FF); return; }
    if ((o & 0x7E) == 0x56) { uint32_t d = bits(op, 2, 0), m = bits(op, 5, 3); uint32_t v = c.R[m]; c.R[d] = (uint32_t)(int32_t)(int16_t)(((v & 0xFF) << 8) | ((v >> 8) & 0xFF)); return; }
    if ((o & 0x70) == 0x60) {
        uint32_t regs = bits(op, 7, 0) | (bit(op, 8) << 15);
        uint32_t addr = c.R[13];
        for (int i = 0; i < 15; ++i) if (regs & (1u << i)) { c.R[i] = c.mem->read32(addr); addr += 4; }
        bool to_pc = regs & (1u << 15);
        uint32_t npc = 0;
        if (to_pc) { npc = c.mem->read32(addr); addr += 4; }
        c.R[13] = addr;
        if (to_pc) c.bxWritePC(npc);
        return;
    }
    if ((o & 0x78) == 0x70) { c.setFault("BKPT"); return; }
    if ((o & 0x78) == 0x78) {
        uint32_t mask = bits(op, 3, 0), firstcond = bits(op, 7, 4);
        if (mask == 0) { return; }
        c.itstate = (uint8_t)((firstcond << 4) | mask);
        return;
    }
    c.setFault("undef-misc16");
}

static void decode16(CPU& c, uint32_t op) {
    uint32_t top = bits(op, 15, 10);
    if (top < 0x10) { decode16_shift_addsub_movcmp(c, op); return; }
    if (top == 0x10) { decode16_data_processing(c, op); return; }
    if (top == 0x11) { decode16_special_data(c, op); return; }
    if (top == 0x12 || top == 0x13) {
        uint32_t t = bits(op, 10, 8), imm = bits(op, 7, 0) << 2;
        uint32_t base = (c.pc_read) & ~3u;
        c.R[t] = c.mem->read32(base + imm);
        return;
    }
    if (top >= 0x14 && top <= 0x27) { load_store_single(c, op); return; }
    if (top == 0x28 || top == 0x29) {
        uint32_t d = bits(op, 10, 8), imm = bits(op, 7, 0) << 2;
        c.R[d] = (c.pc_read & ~3u) + imm;
        return;
    }
    if (top == 0x2A || top == 0x2B) {
        uint32_t d = bits(op, 10, 8), imm = bits(op, 7, 0) << 2;
        c.R[d] = c.R[13] + imm;
        return;
    }
    if (top >= 0x2C && top <= 0x2F) { decode16_misc(c, op); return; }
    if (top == 0x30 || top == 0x31) {
        uint32_t n = bits(op, 10, 8), list = bits(op, 7, 0);
        uint32_t addr = c.R[n];
        for (int i = 0; i < 8; ++i) if (list & (1u << i)) { c.mem->write32(addr, c.R[i]); addr += 4; }
        c.R[n] = c.R[n] + 4 * BitCount(list);
        return;
    }
    if (top == 0x32 || top == 0x33) {
        uint32_t n = bits(op, 10, 8), list = bits(op, 7, 0);
        bool wback = (list & (1u << n)) == 0;
        uint32_t addr = c.R[n];
        for (int i = 0; i < 8; ++i) if (list & (1u << i)) { c.R[i] = c.mem->read32(addr); addr += 4; }
        if (wback) c.R[n] = c.R[n] + 4 * BitCount(list);
        return;
    }
    if (top >= 0x34 && top <= 0x37) {
        uint32_t cond = bits(op, 11, 8);
        if (cond == 0xE) { c.setFault("UDF"); return; }
        if (cond == 0xF) { c.setFault("SVC"); return; }
        int32_t imm = (int32_t)SignExtend(bits(op, 7, 0) << 1, 9);
        if (c.ConditionPassed(cond)) c.branchWritePC(c.pc_read + (uint32_t)imm);
        return;
    }
    if (top == 0x38 || top == 0x39) {
        int32_t imm = (int32_t)SignExtend(bits(op, 10, 0) << 1, 12);
        c.branchWritePC(c.pc_read + (uint32_t)imm);
        return;
    }
    c.setFault("undef16");
}

bool cpu_step(CPU& c) {
    if (c.fault) return false;
    uint32_t addr = c.R[15];
    uint16_t hw1 = c.mem->read16(addr);
    uint32_t hi = hw1 >> 11;
    bool is32 = (hi == 0x1D || hi == 0x1E || hi == 0x1F);
    uint32_t len = is32 ? 4 : 2;

    c.pc_read = addr + 4;
    c.next_seq = addr + len;
    c.branched = false;

    bool in_it = c.InITBlock();
    bool exec = c.ConditionPassed(c.CurrentCond());

    if (is32) {
        uint16_t hw2 = c.mem->read16(addr + 2);
        if (exec) exec_thumb32(c, hw1, hw2);
    } else {
        if (exec) decode16(c, hw1);
    }

    if (in_it) c.ITAdvance();
    if (!c.branched && !c.fault) c.R[15] = c.next_seq;
    c.cycles++;
    return !c.fault;
}

}

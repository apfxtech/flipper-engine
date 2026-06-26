#pragma once

#include <stdint.h>
#include "alu.hpp"

namespace arm {

struct Memory {
    virtual ~Memory() {}
    virtual uint32_t read32(uint32_t addr) = 0;
    virtual uint16_t read16(uint32_t addr) = 0;
    virtual uint8_t read8(uint32_t addr) = 0;
    virtual void write32(uint32_t addr, uint32_t v) = 0;
    virtual void write16(uint32_t addr, uint16_t v) = 0;
    virtual void write8(uint32_t addr, uint8_t v) = 0;
};

enum { MODE_THREAD = 0, MODE_HANDLER = 1 };

constexpr uint32_t FLAG_N = 1u << 31;
constexpr uint32_t FLAG_Z = 1u << 30;
constexpr uint32_t FLAG_C = 1u << 29;
constexpr uint32_t FLAG_V = 1u << 28;
constexpr uint32_t FLAG_Q = 1u << 27;
constexpr uint32_t FLAG_T = 1u << 24;

struct CPU {
    uint32_t R[16];
    uint32_t xpsr;
    uint32_t sp_main;
    uint32_t sp_process;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t basepri;
    uint32_t control;
    int mode;
    uint8_t itstate;

    uint32_t S[32];
    uint32_t fpscr;

    Memory* mem;

    uint32_t pc_read;
    uint32_t next_seq;
    bool branched;
    bool fault;
    const char* fault_msg;

    uint64_t cycles;

    bool N() const { return xpsr & FLAG_N; }
    bool Z() const { return xpsr & FLAG_Z; }
    bool C() const { return xpsr & FLAG_C; }
    bool V() const { return xpsr & FLAG_V; }

    void setFlag(uint32_t bit, bool v) { if (v) xpsr |= bit; else xpsr &= ~bit; }

    uint32_t readR(int n) const { return n == 15 ? pc_read : R[n]; }
    void writeR(int n, uint32_t v) { if (n == 15) branchWritePC(v); else R[n] = v; }

    void branchWritePC(uint32_t addr) { R[15] = addr & ~1u; branched = true; }
    void bxWritePC(uint32_t addr) {
        setFlag(FLAG_T, addr & 1u);
        R[15] = addr & ~1u;
        branched = true;
    }
    void blxWritePC(uint32_t addr) { bxWritePC(addr); }

    bool InITBlock() const { return (itstate & 0x0F) != 0; }
    bool LastInITBlock() const { return (itstate & 0x0F) == 0x08; }
    uint32_t CurrentCond() const {
        if (InITBlock()) return (itstate >> 4) & 0xF;
        return 0xE;
    }

    void ITAdvance() {
        if ((itstate & 0x07) == 0) itstate = 0;
        else itstate = (itstate & 0xE0) | ((itstate << 1) & 0x1F);
    }

    bool ConditionPassed(uint32_t cond) const {
        bool result;
        switch ((cond >> 1) & 0x7) {
            case 0: result = Z(); break;
            case 1: result = C(); break;
            case 2: result = N(); break;
            case 3: result = V(); break;
            case 4: result = C() && !Z(); break;
            case 5: result = N() == V(); break;
            case 6: result = (N() == V()) && !Z(); break;
            default: result = true; break;
        }
        if ((cond & 1) && cond != 0xF) result = !result;
        return result;
    }

    void setFault(const char* msg) { fault = true; fault_msg = msg; }
};

void cpu_reset(CPU& c, uint32_t vtor);
bool cpu_step(CPU& c);

}

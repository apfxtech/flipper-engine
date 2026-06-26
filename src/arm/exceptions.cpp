#include "cpu.hpp"

namespace arm {

static int g_active_count = 0;

int CPU::exc_priority(int n) const {
    if (n == 2) return -2;
    if (n == 3) return -1;
    if (n == EXC_SVCALL) return (int)((scb_shpr[1] >> 24) & 0xFF);
    if (n == EXC_PENDSV) return (int)((scb_shpr[2] >> 16) & 0xFF);
    if (n == EXC_SYSTICK) return (int)((scb_shpr[2] >> 24) & 0xFF);
    if (n >= 16) return (int)nvic_ip[n - 16];
    return 0;
}

int CPU::execution_priority() const {
    int pri = 256;
    if (g_active_count > 0)
        for (int i = 0; i < 512; ++i)
            if (exc_active[i]) { int p = exc_priority(i); if (p < pri) pri = p; }
    if (primask & 1) pri = pri < 0 ? pri : 0;
    if (basepri != 0 && (int)basepri < pri) pri = (int)basepri;
    return pri;
}

int CPU::highest_pending() const {
    int best = -1, bestpri = 256;
    auto consider = [&](int num) { int p = exc_priority(num); if (p < bestpri) { bestpri = p; best = num; } };
    if (pend_svc) consider(EXC_SVCALL);
    if (scb_icsr & (1u << 28)) consider(EXC_PENDSV);
    if (scb_icsr & (1u << 26)) consider(EXC_SYSTICK);
    for (int w = 0; w < 16; ++w) {
        uint32_t en = nvic_iser[w] & nvic_ispr[w];
        while (en) {
            int b = __builtin_ctz(en);
            en &= en - 1;
            consider(16 + w * 32 + b);
        }
    }
    return best;
}

void CPU::exception_entry(int num) {
    bool spsel_thread = (control & CONTROL_SPSEL) && mode == MODE_THREAD;
    uint32_t forcealign = (scb_ccr >> 9) & 1u;
    uint32_t spmask = ~((forcealign ? 4u : 0u));
    uint32_t frameptralign = ((R[13] >> 2) & 1u) & forcealign;
    uint32_t frameptr = (R[13] - 0x20u) & spmask;

    uint32_t ra = R[15];
    uint32_t st = xpsr;
    st = (st & ~(0x3u << 25)) | ((uint32_t)(itstate & 0x3) << 25);
    st = (st & ~(0x3Fu << 10)) | (((uint32_t)(itstate >> 2) & 0x3F) << 10);
    st = (st & ~(1u << 9)) | (frameptralign << 9);

    mem->write32(frameptr + 0x00, R[0]);
    mem->write32(frameptr + 0x04, R[1]);
    mem->write32(frameptr + 0x08, R[2]);
    mem->write32(frameptr + 0x0C, R[3]);
    mem->write32(frameptr + 0x10, R[12]);
    mem->write32(frameptr + 0x14, R[14]);
    mem->write32(frameptr + 0x18, ra);
    mem->write32(frameptr + 0x1C, st);

    if (spsel_thread) sp_process = frameptr; else sp_main = frameptr;

    if (mode == MODE_HANDLER) R[14] = 0xFFFFFFF1u;
    else R[14] = spsel_thread ? 0xFFFFFFFDu : 0xFFFFFFF9u;

    uint32_t vec = mem->read32((vtor & ~0x7Fu) + 4u * (uint32_t)num);
    setFlag(FLAG_T, vec & 1u);
    R[15] = vec & ~1u;
    branched = true;
    mode = MODE_HANDLER;
    R[13] = sp_main;
    control &= ~CONTROL_SPSEL;
    xpsr = (xpsr & ~0x1FFu) | ((uint32_t)num & 0x1FFu);
    itstate = 0;
    control &= ~CONTROL_FPCA;
    if (!exc_active[num]) { exc_active[num] = 1; ++g_active_count; }
}

void CPU::exception_return(uint32_t exc_ret) {
    int returning = (int)(xpsr & 0x1FFu);
    uint32_t frameptr;
    switch (exc_ret & 0xFu) {
        case 0x1: frameptr = sp_main; mode = MODE_HANDLER; control &= ~CONTROL_SPSEL; break;
        case 0x9: frameptr = sp_main; mode = MODE_THREAD; control &= ~CONTROL_SPSEL; break;
        case 0xD: frameptr = sp_process; mode = MODE_THREAD; control |= CONTROL_SPSEL; break;
        default: setFault("bad EXC_RETURN"); return;
    }
    if (exc_active[returning]) { exc_active[returning] = 0; --g_active_count; }

    R[0] = mem->read32(frameptr + 0x00);
    R[1] = mem->read32(frameptr + 0x04);
    R[2] = mem->read32(frameptr + 0x08);
    R[3] = mem->read32(frameptr + 0x0C);
    R[12] = mem->read32(frameptr + 0x10);
    R[14] = mem->read32(frameptr + 0x14);
    uint32_t pc = mem->read32(frameptr + 0x18);
    uint32_t psr = mem->read32(frameptr + 0x1C);
    uint32_t newsp = frameptr + 0x20;
    if (((scb_ccr >> 9) & 1u) && ((psr >> 9) & 1u)) newsp += 4;

    if (mode == MODE_HANDLER || !(control & CONTROL_SPSEL)) sp_main = newsp; else sp_process = newsp;
    R[13] = (mode == MODE_HANDLER || !(control & CONTROL_SPSEL)) ? sp_main : sp_process;

    xpsr = psr & 0xFF00FDFFu;
    itstate = (uint8_t)(((psr >> 25) & 0x3u) | (((psr >> 10) & 0x3Fu) << 2));
    R[15] = pc & ~1u;
    branched = true;
}

void CPU::systick_tick() {
    if (!(systick_ctrl & 1u)) return;
    if (systick_load == 0) return;
    if (systick_val == 0) systick_val = systick_load;
    else {
        systick_val--;
        if (systick_val == 0) {
            systick_ctrl |= (1u << 16);
            if (systick_ctrl & 2u) scb_icsr |= (1u << 26);
        }
    }
}

void CPU::check_exceptions() {
    int n = highest_pending();
    if (n < 0) return;
    if (exc_priority(n) >= execution_priority()) return;
    if (n == EXC_SVCALL) pend_svc = false;
    else if (n == EXC_SYSTICK) scb_icsr &= ~(1u << 26);
    else if (n == EXC_PENDSV) scb_icsr &= ~(1u << 28);
    else nvic_ispr[(n - 16) / 32] &= ~(1u << ((n - 16) & 31));
    exception_entry(n);
}

uint32_t CPU::ppb_read(uint32_t a) {
    switch (a) {
        case 0xE000E010u: { uint32_t v = systick_ctrl; systick_ctrl &= ~(1u << 16); return v; }
        case 0xE000E014u: return systick_load;
        case 0xE000E018u: return systick_val;
        case 0xE000E01Cu: return 0;
        case 0xE000ED04u: return (scb_icsr & ~0x1FFu) | (xpsr & 0x1FFu);
        case 0xE000ED08u: return vtor;
        case 0xE000ED0Cu: return 0xFA050000u;
        case 0xE000ED10u: return scb_scr;
        case 0xE000ED14u: return scb_ccr;
        case 0xE000ED18u: return scb_shpr[0];
        case 0xE000ED1Cu: return scb_shpr[1];
        case 0xE000ED20u: return scb_shpr[2];
        case 0xE000ED88u: return cpacr;
        default: break;
    }
    if (a >= 0xE000E100u && a < 0xE000E140u) return nvic_iser[(a - 0xE000E100u) / 4];
    if (a >= 0xE000E180u && a < 0xE000E1C0u) return nvic_iser[(a - 0xE000E180u) / 4];
    if (a >= 0xE000E200u && a < 0xE000E240u) return nvic_ispr[(a - 0xE000E200u) / 4];
    if (a >= 0xE000E280u && a < 0xE000E2C0u) return nvic_ispr[(a - 0xE000E280u) / 4];
    if (a >= 0xE000E400u && a < 0xE000E7F0u) {
        uint32_t i = (a - 0xE000E400u);
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k) v |= (uint32_t)nvic_ip[i + k] << (8 * k);
        return v;
    }
    return 0;
}

void CPU::ppb_write(uint32_t a, uint32_t v) {
    switch (a) {
        case 0xE000E010u: systick_ctrl = (v & 0x7u) | (systick_ctrl & (1u << 16)); return;
        case 0xE000E014u: systick_load = v & 0xFFFFFFu; return;
        case 0xE000E018u: systick_val = 0; systick_ctrl &= ~(1u << 16); return;
        case 0xE000ED04u:
            if (v & (1u << 28)) scb_icsr |= (1u << 28);
            if (v & (1u << 27)) scb_icsr &= ~(1u << 28);
            if (v & (1u << 26)) scb_icsr |= (1u << 26);
            if (v & (1u << 25)) scb_icsr &= ~(1u << 26);
            return;
        case 0xE000ED08u: vtor = v; return;
        case 0xE000ED0Cu: return;
        case 0xE000ED10u: scb_scr = v; return;
        case 0xE000ED14u: scb_ccr = v; return;
        case 0xE000ED18u: scb_shpr[0] = v; return;
        case 0xE000ED1Cu: scb_shpr[1] = v; return;
        case 0xE000ED20u: scb_shpr[2] = v; return;
        case 0xE000ED88u: cpacr = v; return;
        default: break;
    }
    if (a >= 0xE000E100u && a < 0xE000E140u) { nvic_iser[(a - 0xE000E100u) / 4] |= v; return; }
    if (a >= 0xE000E180u && a < 0xE000E1C0u) { nvic_iser[(a - 0xE000E180u) / 4] &= ~v; return; }
    if (a >= 0xE000E200u && a < 0xE000E240u) { nvic_ispr[(a - 0xE000E200u) / 4] |= v; return; }
    if (a >= 0xE000E280u && a < 0xE000E2C0u) { nvic_ispr[(a - 0xE000E280u) / 4] &= ~v; return; }
    if (a >= 0xE000E400u && a < 0xE000E7F0u) {
        uint32_t i = (a - 0xE000E400u);
        for (int k = 0; k < 4; ++k) nvic_ip[i + k] = (uint8_t)(v >> (8 * k));
        return;
    }
}

}

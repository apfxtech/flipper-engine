#include "../src/arm/alu.hpp"

#include <cstdio>

using namespace arm;

static int g_fail = 0;
static int g_total = 0;

static void check(const char* name, bool ok) {
    ++g_total;
    if (!ok) { ++g_fail; std::printf("FAIL: %s\n", name); }
}

static void chk_add(const char* n, uint32_t x, uint32_t y, bool cin,
                    uint32_t r, bool c, bool v) {
    AddResult a = AddWithCarry(x, y, cin);
    check(n, a.result == r && a.carry == c && a.overflow == v);
}

static void chk_sh(const char* n, ShiftResult a, uint32_t r, bool c) {
    check(n, a.result == r && a.carry == c);
}

int main() {
    chk_add("add 1+2", 1, 2, 0, 3, false, false);
    chk_add("add ffffffff+1", 0xFFFFFFFF, 1, 0, 0, true, false);
    chk_add("add 7fffffff+1 ovf", 0x7FFFFFFF, 1, 0, 0x80000000, false, true);
    chk_add("add 80000000+80000000", 0x80000000, 0x80000000, 0, 0, true, true);
    chk_add("sub 5-3", 5, ~3u, 1, 2, true, false);
    chk_add("sub 3-5", 3, ~5u, 1, 0xFFFFFFFE, false, false);

    chk_sh("lsl 1,1", LSL_C(1, 1), 2, false);
    chk_sh("lsl 80000000,1", LSL_C(0x80000000, 1), 0, true);
    chk_sh("lsl 1,32", LSL_C(1, 32), 0, true);
    chk_sh("lsl 2,32", LSL_C(2, 32), 0, false);
    chk_sh("lsr 1,1", LSR_C(1, 1), 0, true);
    chk_sh("lsr 2,1", LSR_C(2, 1), 1, false);
    chk_sh("lsr 80000000,32", LSR_C(0x80000000, 32), 0, true);
    chk_sh("asr 80000000,1", ASR_C(0x80000000, 1), 0xC0000000, false);
    chk_sh("asr ffffffff,4", ASR_C(0xFFFFFFFF, 4), 0xFFFFFFFF, true);
    chk_sh("ror 1,1", ROR_C(1, 1), 0x80000000, true);
    chk_sh("rrx 2,1", RRX_C(2, true), 0x80000001, false);
    chk_sh("rrx 1,0", RRX_C(1, false), 0, true);

    chk_sh("texp 0x0FF", ThumbExpandImm_C(0x0FF, false), 0xFF, false);
    chk_sh("texp 0x1FF", ThumbExpandImm_C(0x1FF, false), 0x00FF00FF, false);
    chk_sh("texp 0x2FF", ThumbExpandImm_C(0x2FF, false), 0xFF00FF00, false);
    chk_sh("texp 0x3FF", ThumbExpandImm_C(0x3FF, false), 0xFFFFFFFF, false);
    chk_sh("texp 0x400 rot", ThumbExpandImm_C(0x400, false), 0x80000000, true);

    DecodedShift d;
    d = DecodeImmShift(0, 0); check("dec lsl0", d.type == SRType_LSL && d.amount == 0);
    d = DecodeImmShift(1, 0); check("dec lsr32", d.type == SRType_LSR && d.amount == 32);
    d = DecodeImmShift(2, 0); check("dec asr32", d.type == SRType_ASR && d.amount == 32);
    d = DecodeImmShift(3, 0); check("dec rrx", d.type == SRType_RRX && d.amount == 1);
    d = DecodeImmShift(3, 5); check("dec ror5", d.type == SRType_ROR && d.amount == 5);

    check("signext 0xFF@8", SignExtend(0xFF, 8) == 0xFFFFFFFF);
    check("signext 0x7F@8", SignExtend(0x7F, 8) == 0x7F);

    std::printf("%d/%d passed\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

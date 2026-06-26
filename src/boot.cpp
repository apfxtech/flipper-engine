#include "system.hpp"
#include "arm/cpu.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>

static std::unordered_map<uint32_t, std::string> g_mn;

static void load_bin(fze::System& s, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "no %s\n", path); exit(1); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    fread(s.flash.data(), 1, n < (long)s.flash.size() ? n : (long)s.flash.size(), f);
    fclose(f);
}

static void load_dis(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return;
    char line[512];
    while (fgets(line, sizeof line, f)) {
        char* colon = strchr(line, ':');
        if (!colon) continue;
        char* p = line; while (*p == ' ') ++p;
        char* end = nullptr;
        unsigned long a = strtoul(p, &end, 16);
        if (end != colon) continue;
        char* t = strchr(colon, '\t');
        if (!t) continue;
        char* t2 = strchr(t + 1, '\t');
        std::string str = t2 ? t2 + 1 : "";
        if (!str.empty() && str.back() == '\n') str.pop_back();
        g_mn[(uint32_t)a] = str;
    }
    fclose(f);
}

static const char* dis(uint32_t pc) {
    auto it = g_mn.find(pc);
    return it == g_mn.end() ? "?" : it->second.c_str();
}

int main(int argc, char** argv) {
    const char* bin = "reference/firmware.bin";
    uint64_t budget = 200000000;
    for (int i = 1; i < argc; ++i)
        if (!strcmp(argv[i], "-n") && i + 1 < argc) budget = strtoull(argv[++i], 0, 0);

    fze::System sys;
    load_bin(sys, bin);
    load_dis("reference/firmware.dis");

    arm::CPU c;
    c.mem = &sys;
    sys.cycles = &c.cycles;
    sys.cpu = &c;
    arm::cpu_reset(c, fze::FLASH_BASE);
    printf("reset PC=%08X SP=%08X\n", c.R[15], c.R[13]);

    int frame_count = 0;
    bool dumped = false;
    sys.bridge.on_frame = [&](const uint8_t* px, int w, int h) {
        ++frame_count;
        int set = 0;
        for (int i = 0; i < w * h; ++i) set += px[i];
        if (dumped || set < 300) return;
        dumped = true;
        printf("\n=== FRAME #%d, %d px set, cyc %llu (on=%d) ===\n", frame_count, set,
               (unsigned long long)c.cycles, sys.lcd.on);
        for (int y = 0; y < h; y += 2) {
            for (int x = 0; x < w; ++x) {
                int top = px[y * w + x], bot = (y + 1 < h) ? px[(y + 1) * w + x] : 0;
                const char* ch = top && bot ? "█" : top ? "▀" : bot ? "▄" : " ";
                fputs(ch, stdout);
            }
            putchar('\n');
        }
        fflush(stdout);
    };

    uint32_t window[256];
    int wi = 0;
    uint64_t n = 0;
    bool hit_reset = false;
    for (; n < budget; ++n) {
        if (!hit_reset && c.R[15] == 0x080128A8u) {
            hit_reset = true;
            printf("CRASH at step %llu ip=%08X LR=%08X : %s\n", (unsigned long long)n, c.R[12],
                   c.R[14], dis((c.R[14] & ~1u) - 4));
            printf("  heap globals:");
            for (uint32_t a = 0x200009B0u; a <= 0x200009E8u; a += 4) printf(" [%X]=%08X", a & 0xFFF, sys.read32(a));
            printf("\n");
        }
        if (c.R[15] == 0x08014808u && c.R[0] >= 0x10000u) {
            printf("BIG-MALLOC(%u) at step %llu LR=%08X : %s\n", c.R[0], (unsigned long long)n,
                   c.R[14], dis((c.R[14] & ~1u) - 4));
        }
        if (!sys.core2_started && (n & 63) == 0) {
            bool was = sys.core2_started;
            sys.core2_tick();
            if (!was && sys.core2_started)
                printf("CORE2 ready injected at step %llu PC=%08X\n", (unsigned long long)n, c.R[15]);
        }
        window[wi++ & 255] = c.R[15];
        if (!arm::cpu_step(c)) {
            printf("\nFAULT step %llu PC=%08X: %s\n  %s\n", (unsigned long long)n, c.R[15],
                   c.fault_msg ? c.fault_msg : "", dis(c.R[15]));
            return 2;
        }
        if ((n % 10000000) == 0 && n > 0)
            printf("[%lluM] PC=%08X : %s\n", (unsigned long long)(n / 1000000), c.R[15], dis(c.R[15]));
        if (false && (n & 0xFFFFF) == 0xFFFFF) {
            uint32_t lo = 0xFFFFFFFF, hi = 0;
            for (int k = 0; k < 256; ++k) { if (window[k] < lo) lo = window[k]; if (window[k] > hi) hi = window[k]; }
            if (hi - lo < 0x60) {
                printf("\nSPIN at step %llu, PC range %08X..%08X\n", (unsigned long long)n, lo, hi);
                uint32_t seen[64]; int sc = 0;
                uint32_t plo = 0xFFFFFFFF, phi = 0;
                for (int k = 0; k < 2000000 && sc < 64; ++k) {
                    sys.last_read = 0;
                    uint32_t pc0 = c.R[15];
                    arm::cpu_step(c);
                    if (pc0 < plo) plo = pc0;
                    if (pc0 > phi) phi = pc0;
                    uint32_t r = sys.last_read;
                    if (r && r != 0xE0001004u && (r < 0xE000E000u || r >= 0xE000F000u)) {
                        bool dup = false;
                        for (int j = 0; j < sc; ++j) if (seen[j] == r) dup = true;
                        if (!dup && sc < 64) seen[sc++] = r;
                    }
                }
                printf("  не-таймерные MMIO адреса за 2M шагов (PC %08X..%08X):\n", plo, phi);
                for (int j = 0; j < sc; ++j) printf("    %08X\n", seen[j]);
                return 0;
            }
        }
    }
    printf("ran %llu, PC=%08X : %s\n", (unsigned long long)n, c.R[15], dis(c.R[15]));
    printf("display: spi2_dr=%llu routed=%llu frames=%d lcd.on=%d page=%d col=%d cs(C11)=%d dc(B1)=%d\n",
           (unsigned long long)sys.dbg_spi2_dr, (unsigned long long)sys.dbg_disp_routed,
           frame_count, sys.lcd.on, sys.lcd.page, sys.lcd.col,
           (sys.gpio[2].odr >> 11) & 1, (sys.gpio[1].odr >> 1) & 1);
    return 0;
}

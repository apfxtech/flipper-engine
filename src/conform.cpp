#include "flatmem.hpp"
#include "arm/cpu.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

static void load_bin(fze::FlatMem& m, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "no %s\n", path); exit(1); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    fread(m.flash.data(), 1, n < (long)m.flash.size() ? n : (long)m.flash.size(), f);
    fclose(f);
}

static void load_dis(const char* path, std::unordered_map<uint32_t, std::string>& mn) {
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "no %s\n", path); exit(1); }
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
        std::string s = t2 ? t2 + 1 : "";
        if (!s.empty() && s.back() == '\n') s.pop_back();
        mn[(uint32_t)a] = s;
    }
    fclose(f);
}

int main(int argc, char** argv) {
    const char* bin = "reference/firmware.bin";
    const char* dis = "reference/firmware.dis";
    uint64_t budget = 100000000;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) budget = strtoull(argv[++i], 0, 0);
        else if (!strcmp(argv[i], "-bin") && i + 1 < argc) bin = argv[++i];
        else if (!strcmp(argv[i], "-dis") && i + 1 < argc) dis = argv[++i];
    }

    fze::FlatMem mem;
    load_bin(mem, bin);
    std::unordered_map<uint32_t, std::string> mn;
    load_dis(dis, mn);
    printf("boundaries: %zu\n", mn.size());

    arm::CPU c;
    c.mem = &mem;
    arm::cpu_reset(c, fze::FLASH_BASE);
    printf("reset PC=%08X SP=%08X\n", c.R[15], c.R[13]);

    uint32_t prev = c.R[15];
    uint64_t n = 0;
    for (; n < budget; ++n) {
        uint32_t pc = c.R[15];
        if (pc >= fze::FLASH_BASE && pc < fze::FLASH_BASE + fze::FLASH_SIZE && mn.find(pc) == mn.end()) {
            auto it = mn.find(prev);
            printf("\nDESYNC step %llu PC=%08X not a boundary\n", (unsigned long long)n, pc);
            printf("  prev PC=%08X : %s\n", prev, it != mn.end() ? it->second.c_str() : "?");
            printf("  r0=%08X r1=%08X r2=%08X r3=%08X r12=%08X sp=%08X lr=%08X\n",
                   c.R[0], c.R[1], c.R[2], c.R[3], c.R[12], c.R[13], c.R[14]);
            return 3;
        }
        prev = pc;
        if (!arm::cpu_step(c)) {
            auto it = mn.find(prev);
            printf("\nFAULT step %llu PC=%08X: %s\n  at %08X : %s\n", (unsigned long long)n, c.R[15],
                   c.fault_msg ? c.fault_msg : "", prev, it != mn.end() ? it->second.c_str() : "?");
            return 2;
        }
    }
    printf("ran %llu, no desync, PC=%08X\n", (unsigned long long)n, c.R[15]);
    return 0;
}

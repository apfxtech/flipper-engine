#pragma once

#include "arm/cpu.hpp"
#include <vector>
#include <unordered_map>
#include <cstring>

namespace fze {

constexpr uint32_t FLASH_BASE = 0x08000000u;
constexpr uint32_t FLASH_SIZE = 0x00100000u;
constexpr uint32_t SRAM_BASE = 0x20000000u;
constexpr uint32_t SRAM_SIZE = 0x00040000u;

struct FlatMem : arm::Memory {
    std::vector<uint8_t> flash;
    std::vector<uint8_t> sram;
    std::unordered_map<uint32_t, uint32_t> io;

    FlatMem() : flash(FLASH_SIZE, 0xFF), sram(SRAM_SIZE, 0) {}

    uint8_t* map(uint32_t a, uint32_t n) {
        if (a + n <= FLASH_SIZE) return &flash[a];
        if (a >= FLASH_BASE && a + n <= FLASH_BASE + FLASH_SIZE) return &flash[a - FLASH_BASE];
        if (a >= SRAM_BASE && a + n <= SRAM_BASE + SRAM_SIZE) return &sram[a - SRAM_BASE];
        return nullptr;
    }

    uint32_t read32(uint32_t a) override { if (auto p = map(a, 4)) { uint32_t v; memcpy(&v, p, 4); return v; } auto it = io.find(a & ~3u); return it == io.end() ? 0 : it->second; }
    uint16_t read16(uint32_t a) override { if (auto p = map(a, 2)) { uint16_t v; memcpy(&v, p, 2); return v; } return (uint16_t)(read32(a & ~3u) >> ((a & 2) * 8)); }
    uint8_t read8(uint32_t a) override { if (auto p = map(a, 1)) return *p; return (uint8_t)(read32(a & ~3u) >> ((a & 3) * 8)); }

    void write32(uint32_t a, uint32_t v) override { if (a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE) return; if (auto p = map(a, 4)) { memcpy(p, &v, 4); return; } io[a & ~3u] = v; }
    void write16(uint32_t a, uint16_t v) override { if (auto p = map(a, 2)) { memcpy(p, &v, 2); return; } uint32_t w = read32(a & ~3u), s = (a & 2) * 8; io[a & ~3u] = (w & ~(0xFFFFu << s)) | ((uint32_t)v << s); }
    void write8(uint32_t a, uint8_t v) override { if (auto p = map(a, 1)) { *p = v; return; } uint32_t w = read32(a & ~3u), s = (a & 3) * 8; io[a & ~3u] = (w & ~(0xFFu << s)) | ((uint32_t)v << s); }
};

}

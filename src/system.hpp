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

constexpr uint32_t GPIO_BASE = 0x48000000u;
constexpr uint32_t RCC_BASE = 0x58000000u;
constexpr uint32_t SPI1_BASE = 0x40013000u;
constexpr uint32_t SPI2_BASE = 0x40003800u;
constexpr uint32_t IPCC_BASE = 0x58000C00u;
constexpr uint32_t RNG_BASE = 0x58001000u;
constexpr uint32_t HSEM_BASE = 0x58001400u;
constexpr uint32_t HSEM_CORE1_LOCK = 0x80000400u;
constexpr uint32_t TL_REF_TABLE = 0x20030000u;

constexpr int IPCC_C1_RX_IRQn = 44;
constexpr uint32_t IPCC_SYS_CHANNEL = 0x2u;

struct GpioPort {
    uint32_t moder = 0;
    uint32_t odr = 0;
    uint32_t idr_input = 0;
    uint32_t regs[16] = {0};
};

struct System : arm::Memory {
    std::vector<uint8_t> flash;
    std::vector<uint8_t> sram;
    std::unordered_map<uint32_t, uint32_t> io;

    uint32_t rcc_cr = 0;
    uint32_t rcc_cfgr = 0;
    uint32_t rcc_bdcr = 0;
    uint32_t rcc_csr = 0;
    uint32_t rcc_crrcr = 0;

    uint32_t ipcc_c1cr = 0, ipcc_c1mr = 0, ipcc_c1scr = 0, ipcc_c1toc2sr = 0;
    uint32_t ipcc_c2cr = 0, ipcc_c2mr = 0, ipcc_c2scr = 0, ipcc_c2toc1sr = 0;
    bool core2_started = false;

    struct SpiDev {
        bool rx_valid = false;
        uint8_t rx_byte = 0;
        uint32_t cr1 = 0, cr2 = 0;
    };
    SpiDev spi1, spi2;

    uint32_t rng_state = 0x2545F491u;
    uint32_t rng_next() {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    }

    GpioPort gpio[8];

    System() : flash(FLASH_SIZE, 0xFF), sram(SRAM_SIZE, 0) {
        rcc_cr = recompute_cr(0x00000061u);
        gpio[1].idr_input = (1u << 10) | (1u << 11) | (1u << 12);
        gpio[2].idr_input = (1u << 6) | (1u << 13);
        gpio[7].idr_input = 0;
    }

    static uint32_t recompute_cr(uint32_t cr) {
        auto f = [&](int on, int rdy) { if (cr & (1u << on)) cr |= (1u << rdy); else cr &= ~(1u << rdy); };
        f(0, 1); f(8, 10); f(16, 17); f(24, 25); f(26, 27);
        return cr;
    }

    uint8_t* map(uint32_t a, uint32_t n) {
        if (a + n <= FLASH_SIZE) return &flash[a];
        if (a >= FLASH_BASE && a + n <= FLASH_BASE + FLASH_SIZE) return &flash[a - FLASH_BASE];
        if (a >= SRAM_BASE && a + n <= SRAM_BASE + SRAM_SIZE) return &sram[a - SRAM_BASE];
        return nullptr;
    }

    bool is_gpio(uint32_t a, int& port, uint32_t& off) {
        if (a >= GPIO_BASE && a < GPIO_BASE + 8 * 0x400) {
            port = (a - GPIO_BASE) / 0x400;
            off = (a - GPIO_BASE) % 0x400;
            return true;
        }
        return false;
    }

    uint32_t gpio_read(int p, uint32_t off) {
        GpioPort& g = gpio[p];
        switch (off) {
            case 0x00: return g.moder;
            case 0x10: return g.odr | g.idr_input;
            case 0x14: return g.odr;
            default: return g.regs[off / 4 & 0xF];
        }
    }
    void gpio_write(int p, uint32_t off, uint32_t v) {
        GpioPort& g = gpio[p];
        switch (off) {
            case 0x00: g.moder = v; break;
            case 0x14: g.odr = v; break;
            case 0x18: g.odr = (g.odr | (v & 0xFFFF)) & ~(v >> 16); break;
            case 0x28: g.odr &= ~v; break;
            default: g.regs[off / 4 & 0xF] = v; break;
        }
    }

    uint32_t rcc_read(uint32_t off) {
        switch (off) {
            case 0x00: return rcc_cr;
            case 0x08: return rcc_cfgr;
            case 0x90: return rcc_bdcr;
            case 0x94: return rcc_csr;
            case 0x98: return rcc_crrcr;
            default: { auto it = io.find(RCC_BASE + off); return it == io.end() ? 0 : it->second; }
        }
    }
    void rcc_write(uint32_t off, uint32_t v) {
        switch (off) {
            case 0x00: rcc_cr = recompute_cr(v); break;
            case 0x08: rcc_cfgr = (v & ~0xCu) | ((v & 0x3u) << 2); break;
            case 0x90: rcc_bdcr = (v & ~0x2u) | ((v & 0x1u) << 1); break;
            case 0x94: rcc_csr = (v & ~0x2u) | ((v & 0x1u) << 1); break;
            case 0x98: rcc_crrcr = (v & ~0x2u) | ((v & 0x1u) << 1); break;
            default: io[RCC_BASE + off] = v; break;
        }
    }

    uint32_t ipcc_read(uint32_t off) {
        switch (off) {
            case 0x00: return ipcc_c1cr;
            case 0x04: return ipcc_c1mr;
            case 0x08: return ipcc_c1scr;
            case 0x0C: return ipcc_c1toc2sr;
            case 0x10: return ipcc_c2cr;
            case 0x14: return ipcc_c2mr;
            case 0x18: return ipcc_c2scr;
            case 0x1C: return ipcc_c2toc1sr;
            default: return 0;
        }
    }
    void ipcc_write(uint32_t off, uint32_t v) {
        switch (off) {
            case 0x00: ipcc_c1cr = v; break;
            case 0x04: ipcc_c1mr = v; break;
            case 0x08:
                ipcc_c2toc1sr &= ~(v & 0x3Fu);
                ipcc_c1toc2sr |= (v >> 16) & 0x3Fu;
                break;
            case 0x0C: ipcc_c1toc2sr = v; break;
            case 0x10: ipcc_c2cr = v; break;
            case 0x14: ipcc_c2mr = v; break;
            case 0x18: ipcc_c2scr = v; break;
            case 0x1C: ipcc_c2toc1sr = v; break;
            default: break;
        }
    }

    SpiDev* spi_for(uint32_t a, uint32_t& off) {
        if (a >= SPI1_BASE && a < SPI1_BASE + 0x400) { off = a - SPI1_BASE; return &spi1; }
        if (a >= SPI2_BASE && a < SPI2_BASE + 0x400) { off = a - SPI2_BASE; return &spi2; }
        return nullptr;
    }
    uint32_t spi_read(SpiDev& d, uint32_t off) {
        switch (off) {
            case 0x00: return d.cr1;
            case 0x04: return d.cr2;
            case 0x08: return 0x2u | (d.rx_valid ? (0x1u | (1u << 9)) : 0u);
            case 0x0C: { d.rx_valid = false; return d.rx_byte; }
            default: return 0;
        }
    }
    void spi_write(SpiDev& d, uint32_t off, uint32_t v) {
        switch (off) {
            case 0x00: d.cr1 = v; break;
            case 0x04: d.cr2 = v; break;
            case 0x0C: d.rx_byte = 0x00; d.rx_valid = true; break;
            default: break;
        }
    }

    void core2_tick() {
        if (core2_started || !cpu) return;
        uint32_t p_sys = read32(TL_REF_TABLE + 12);
        uint32_t p_mm = read32(TL_REF_TABLE + 16);
        if (!p_sys || !p_mm) return;
        uint32_t sys_queue = read32(p_sys + 4);
        uint32_t spare = read32(p_mm + 4);
        if (!sys_queue || !spare) return;
        if (ipcc_c1mr & IPCC_SYS_CHANNEL) return;
        uint32_t w = IPCC_C1_RX_IRQn / 32, b = IPCC_C1_RX_IRQn % 32;
        if (!(cpu->nvic_iser[w] & (1u << b))) return;
        if (read32(sys_queue) == 0) return;

        write32(spare + 0, sys_queue);
        write32(spare + 4, sys_queue);
        write8(spare + 8, 0x12);
        write8(spare + 9, 0xFF);
        write8(spare + 10, 6);
        write8(spare + 11, 0x00);
        write8(spare + 12, 0x92);
        write8(spare + 13, 0x00);
        write8(spare + 14, 0x00);
        write8(spare + 15, 0x00);
        write32(sys_queue + 0, spare);
        write32(sys_queue + 4, spare);

        ipcc_c2toc1sr |= IPCC_SYS_CHANNEL;
        cpu->nvic_ispr[w] |= (1u << b);
        core2_started = true;
    }

    bool trace_io = false;
    uint32_t last_read = 0;
    uint64_t* cycles = nullptr;
    arm::CPU* cpu = nullptr;

    uint32_t mmio_read_word(uint32_t a) {
        last_read = a;
        int p; uint32_t off;
        if (is_gpio(a, p, off)) return gpio_read(p, off);
        { uint32_t so; if (SpiDev* sd = spi_for(a, so)) return spi_read(*sd, so); }
        if (a >= IPCC_BASE && a < IPCC_BASE + 0x400) return ipcc_read(a - IPCC_BASE);
        if (a >= RNG_BASE && a < RNG_BASE + 0x400) {
            uint32_t off = a - RNG_BASE;
            if (off == 0x04) return 0x1u;
            if (off == 0x08) return rng_next();
            auto it = io.find(a); return it == io.end() ? 0 : it->second;
        }
        if (a >= HSEM_BASE && a < HSEM_BASE + 0x400) {
            uint32_t off = a - HSEM_BASE;
            if (off >= 0x80 && off < 0x100) return HSEM_CORE1_LOCK;
            if (off >= 0x00 && off < 0x80) { auto it = io.find(a); return it == io.end() ? 0 : it->second; }
            auto it = io.find(a); return it == io.end() ? 0 : it->second;
        }
        if (a >= RCC_BASE && a < RCC_BASE + 0x400) return rcc_read(a - RCC_BASE);
        if (a == 0xE0001004u) return cycles ? (uint32_t)*cycles : 0;
        if (a == 0x4001381Cu || a == 0x4000801Cu) return 0x006000D0u;
        if (a >= 0xE000E000u && a < 0xE000F000u && cpu) return cpu->ppb_read(a);
        auto it = io.find(a);
        return it == io.end() ? 0 : it->second;
    }
    void mmio_write_word(uint32_t a, uint32_t v) {
        int p; uint32_t off;
        if (is_gpio(a, p, off)) { gpio_write(p, off, v); return; }
        { uint32_t so; if (SpiDev* sd = spi_for(a, so)) { spi_write(*sd, so, v); return; } }
        if (a >= IPCC_BASE && a < IPCC_BASE + 0x400) { ipcc_write(a - IPCC_BASE, v); return; }
        if (a >= RCC_BASE && a < RCC_BASE + 0x400) { rcc_write(a - RCC_BASE, v); return; }
        if (a >= 0xE000E000u && a < 0xE000F000u && cpu) { cpu->ppb_write(a, v); return; }
        io[a] = v;
    }

    uint32_t read32(uint32_t a) override { if (auto q = map(a, 4)) { uint32_t v; memcpy(&v, q, 4); return v; } return mmio_read_word(a & ~3u); }
    uint16_t read16(uint32_t a) override { if (auto q = map(a, 2)) { uint16_t v; memcpy(&v, q, 2); return v; } return (uint16_t)(mmio_read_word(a & ~3u) >> ((a & 2) * 8)); }
    uint8_t read8(uint32_t a) override { if (auto q = map(a, 1)) return *q; return (uint8_t)(mmio_read_word(a & ~3u) >> ((a & 3) * 8)); }

    void write32(uint32_t a, uint32_t v) override { if (a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE) return; if (auto q = map(a, 4)) { memcpy(q, &v, 4); return; } mmio_write_word(a & ~3u, v); }
    void write16(uint32_t a, uint16_t v) override { if (auto q = map(a, 2)) { memcpy(q, &v, 2); return; } uint32_t w = mmio_read_word(a & ~3u), s = (a & 2) * 8; mmio_write_word(a & ~3u, (w & ~(0xFFFFu << s)) | ((uint32_t)v << s)); }
    void write8(uint32_t a, uint8_t v) override { if (auto q = map(a, 1)) { *q = v; return; } uint32_t w = mmio_read_word(a & ~3u), s = (a & 3) * 8; mmio_write_word(a & ~3u, (w & ~(0xFFu << s)) | ((uint32_t)v << s)); }
};

}

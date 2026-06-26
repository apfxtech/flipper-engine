#pragma once

#include <cstdint>
#include <cstring>

namespace fze {

struct St7565 {
    static constexpr int W = 128;
    static constexpr int H = 64;
    static constexpr int PAGES = 8;
    static constexpr int COLS = 132;

    uint8_t gdram[PAGES * COLS] = {0};
    int page = 0;
    int col = 0;
    int start_line = 0;
    bool on = false;
    bool adc_reverse = true;
    bool com_reverse = false;
    bool dirty = false;

    bool command(uint8_t b) {
        bool frame = false;
        if (b >= 0xB0 && b <= 0xB7) {
            int np = b & 0x07;
            if (np < page && dirty) { frame = true; dirty = false; }
            page = np;
        } else if (b >= 0x10 && b <= 0x1F) {
            col = (col & 0x0F) | ((b & 0x0F) << 4);
        } else if (b <= 0x0F) {
            col = (col & 0xF0) | (b & 0x0F);
        } else if (b == 0xAF) {
            on = true;
        } else if (b == 0xAE) {
            on = false;
        } else if (b == 0xA0) {
            adc_reverse = false;
        } else if (b == 0xA1) {
            adc_reverse = true;
        } else if (b == 0xC0) {
            com_reverse = false;
        } else if (b == 0xC8) {
            com_reverse = true;
        } else if (b >= 0x40 && b <= 0x7F) {
            start_line = b & 0x3F;
        }
        return frame;
    }

    void data(uint8_t b) {
        if (col >= 0 && col < COLS) gdram[page * COLS + col] = b;
        if (col < COLS) col++;
        dirty = true;
    }

    void render(uint8_t* px) const {
        for (int sy = 0; sy < H; ++sy) {
            for (int sx = 0; sx < W; ++sx) {
                int c = adc_reverse ? (W - 1 - sx) : sx;
                int y = com_reverse ? (H - 1 - sy) : sy;
                int p = y >> 3;
                int bit = y & 7;
                uint8_t byte = gdram[p * COLS + c];
                px[sy * W + sx] = (byte >> bit) & 1u;
            }
        }
    }
};

}

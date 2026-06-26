#pragma once

#include <cstdint>
#include <functional>

namespace fze {

enum Button : uint32_t {
    ButtonUp = 1u << 0,
    ButtonDown = 1u << 1,
    ButtonRight = 1u << 2,
    ButtonLeft = 1u << 3,
    ButtonOk = 1u << 4,
    ButtonBack = 1u << 5,
};

struct CoreBridge {
    std::function<void(const uint8_t* pixels, int w, int h)> on_frame;
    std::function<uint32_t()> poll_buttons;
};

}

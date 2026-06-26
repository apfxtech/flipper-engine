#include "system.hpp"
#include "arm/cpu.hpp"

#include <SFML/Graphics.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>

static void load_bin(fze::System& s, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "no %s\n", path); exit(1); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    fread(s.flash.data(), 1, n < (long)s.flash.size() ? n : (long)s.flash.size(), f);
    fclose(f);
}

int main(int argc, char** argv) {
    const char* bin = "reference/firmware.bin";
    unsigned scale = 4;
    long batch = 8000000;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-bin") && i + 1 < argc) bin = argv[++i];
        else if (!strcmp(argv[i], "-scale") && i + 1 < argc) scale = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-batch") && i + 1 < argc) batch = atol(argv[++i]);
    }

    fze::System sys;
    load_bin(sys, bin);

    arm::CPU c;
    c.mem = &sys;
    sys.cycles = &c.cycles;
    sys.cpu = &c;
    arm::cpu_reset(c, fze::FLASH_BASE);

    static uint8_t pixels[fze::St7565::W * fze::St7565::H];
    bool have_frame = false;
    uint32_t buttons = 0;

    sys.bridge.on_frame = [&](const uint8_t* px, int w, int h) {
        memcpy(pixels, px, (size_t)w * h);
        have_frame = true;
    };
    sys.bridge.poll_buttons = [&]() { return buttons; };

    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(fze::St7565::W * scale, fze::St7565::H * scale)),
        sf::String("Flipper Zero - flipper-engine"));
    window.setFramerateLimit(60);

    sf::Image image(sf::Vector2u(fze::St7565::W, fze::St7565::H), sf::Color(0, 0, 0));
    sf::Texture texture(sf::Vector2u(fze::St7565::W, fze::St7565::H));
    sf::Sprite sprite(texture);
    sprite.setScale(sf::Vector2f((float)scale, (float)scale));

    const sf::Color on_color(0x00, 0x00, 0x00);
    const sf::Color off_color(0xFF, 0x99, 0x00);

    bool faulted = false;
    while (window.isOpen()) {
        while (const std::optional ev = window.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) window.close();
        }

        using K = sf::Keyboard::Key;
        buttons = 0;
        if (sf::Keyboard::isKeyPressed(K::Up)) buttons |= fze::ButtonUp;
        if (sf::Keyboard::isKeyPressed(K::Down)) buttons |= fze::ButtonDown;
        if (sf::Keyboard::isKeyPressed(K::Left)) buttons |= fze::ButtonLeft;
        if (sf::Keyboard::isKeyPressed(K::Right)) buttons |= fze::ButtonRight;
        if (sf::Keyboard::isKeyPressed(K::Enter) || sf::Keyboard::isKeyPressed(K::Space))
            buttons |= fze::ButtonOk;
        if (sf::Keyboard::isKeyPressed(K::Backspace) || sf::Keyboard::isKeyPressed(K::Escape))
            buttons |= fze::ButtonBack;

        if (!faulted) {
            for (long i = 0; i < batch; ++i) {
                if (!sys.core2_started && (i & 63) == 0) sys.core2_tick();
                if (!arm::cpu_step(c)) {
                    printf("FAULT PC=%08X: %s\n", c.R[15], c.fault_msg ? c.fault_msg : "");
                    faulted = true;
                    break;
                }
            }
        }

        if (have_frame) {
            for (int y = 0; y < fze::St7565::H; ++y)
                for (int x = 0; x < fze::St7565::W; ++x)
                    image.setPixel(
                        sf::Vector2u(x, y), pixels[y * fze::St7565::W + x] ? on_color : off_color);
            texture.update(image);
        }

        window.clear(sf::Color(40, 40, 40));
        window.draw(sprite);
        window.display();
    }
    return 0;
}

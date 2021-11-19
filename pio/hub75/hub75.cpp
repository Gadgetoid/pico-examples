/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <cstdint>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hub75.pio.h"

#define DATA_BASE_PIN 0
#define DATA_N_PINS 6
#define ROWSEL_BASE_PIN 6
#define ROWSEL_N_PINS 5
#define CLK_PIN 11
#define STROBE_PIN 12
#define OEN_PIN 13

const bool CLK_POLARITY = 1;
const bool STB_POLARITY = 1;
const bool OE_POLARITY = 0;

#define WIDTH 64
#define HEIGHT 64

// This gamma table is used to correct our 8-bit (0-255) colours up to 11-bit,
// allowing us to gamma correct without losing dynamic range.
constexpr uint16_t GAMMA_12BIT[256] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 47, 50,
    52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 91, 94,
     98, 101, 105, 108, 112, 115, 119, 123, 127, 131, 135, 139, 143, 147, 151, 155,
    160, 164, 169, 173, 178, 183, 187, 192, 197, 202, 207, 212, 217, 223, 228, 233,
    239, 244, 250, 255, 261, 267, 273, 279, 285, 291, 297, 303, 309, 316, 322, 328,
    335, 342, 348, 355, 362, 369, 376, 383, 390, 397, 404, 412, 419, 427, 434, 442,
    449, 457, 465, 473, 481, 489, 497, 505, 513, 522, 530, 539, 547, 556, 565, 573,
    582, 591, 600, 609, 618, 628, 637, 646, 656, 665, 675, 685, 694, 704, 714, 724,
    734, 744, 755, 765, 775, 786, 796, 807, 817, 828, 839, 850, 861, 872, 883, 894,
    905, 917, 928, 940, 951, 963, 975, 987, 998, 1010, 1022, 1035, 1047, 1059, 1071, 1084,
    1096, 1109, 1122, 1135, 1147, 1160, 1173, 1186, 1199, 1213, 1226, 1239, 1253, 1266, 1280, 1294,
    1308, 1321, 1335, 1349, 1364, 1378, 1392, 1406, 1421, 1435, 1450, 1465, 1479, 1494, 1509, 1524,
    1539, 1554, 1570, 1585, 1600, 1616, 1631, 1647, 1663, 1678, 1694, 1710, 1726, 1743, 1759, 1775,
    1791, 1808, 1824, 1841, 1858, 1875, 1891, 1908, 1925, 1943, 1960, 1977, 1994, 2012, 2029, 2047};


// We don't *need* to make Pixel a fancy struct with RGB values, but it helps.
#pragma pack(push, 1)
struct alignas(4) Pixel {
    uint16_t _;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    constexpr Pixel() : _(0), r(0), g(0), b(0) {};
    constexpr Pixel(uint8_t r, uint8_t g, uint8_t b) : _(0), r(GAMMA_12BIT[r]), g(GAMMA_12BIT[g]), b(GAMMA_12BIT[b]) {};
};
#pragma pack(pop)

Pixel front_buffer[WIDTH * HEIGHT];
uint32_t back_buffer[WIDTH * HEIGHT * 2];

uint dma_channel = 0;
bool do_flip = false;
uint bit = 0;
uint row = 0;

PIO pio = pio0;
uint sm_data = 0;
uint sm_row = 1;

uint data_prog_offs = 0;
uint row_prog_offs = 0;


inline uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

// Basic function to convert Hue, Saturation and Value to an RGB colour
Pixel hsv_to_rgb(float h, float s, float v) {
    if(h < 0.0f) {
        h = 1.0f + fmod(h, 1.0f);
    }

    int i = int(h * 6);
    float f = h * 6 - i;

    v = v * 255.0f;

    float sv = s * v;
    float fsv = f * sv;

    auto p = uint8_t(-sv + v);
    auto q = uint8_t(-fsv + v);
    auto t = uint8_t(fsv - sv + v);

    uint8_t bv = uint8_t(v);

    switch (i % 6) {
        default:
        case 0: return Pixel(bv, t, p);
        case 1: return Pixel(q, bv, p);
        case 2: return Pixel(p, bv, t);
        case 3: return Pixel(p, q, bv);
        case 4: return Pixel(t, p, bv);
        case 5: return Pixel(bv, p, q);
    }
}

void set_rgb(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    int offset = 0;
    if(y >= 32) {
        y -= 32;
        offset = (y * WIDTH + x) * 2;
        offset += 1;
    } else {
        offset = (y * WIDTH + x) * 2;
    }
    front_buffer[offset] = Pixel(r, g, b);
}

void set_hsv(uint8_t x, uint8_t y, float h, float s, float v) {
    int offset = 0;
    if(y >= 32) {
        y -= 32;
        offset = (y * WIDTH + x) * 2;
        offset += 1;
    } else {
        offset = (y * WIDTH + x) * 2;
    }
    front_buffer[offset] = hsv_to_rgb(h, s, v);
}

// Required for FM6126A-based displays which need some register config/init to work properly
void FM6126A_write_register(uint16_t value, uint8_t position) {
    uint8_t threshold = WIDTH - position;
    for(auto i = 0u; i < WIDTH; i++) {
        auto j = i % 16;
        bool b = value & (1 << j);
        gpio_put(DATA_BASE_PIN + 0, b);
        gpio_put(DATA_BASE_PIN + 1, b);
        gpio_put(DATA_BASE_PIN + 2, b);
        gpio_put(DATA_BASE_PIN + 3, b);
        gpio_put(DATA_BASE_PIN + 4, b);
        gpio_put(DATA_BASE_PIN + 5, b);

        // Assert strobe/latch if i > threshold
        // This somehow indicates to the FM6126A which register we want to write :|
        gpio_put(STROBE_PIN, i > threshold);
        gpio_put(CLK_PIN, CLK_POLARITY);
        sleep_us(10);
        gpio_put(CLK_PIN, !CLK_POLARITY);
    }
}

void setup_pin(uint pin) {
    gpio_init(pin); gpio_set_function(pin, GPIO_FUNC_SIO); gpio_set_dir(pin, true);
}

void flip() {
    do_flip = true;
}

void __isr dma_complete() {
    if (do_flip && bit == 0 && row == 0) {
        memcpy((uint8_t *)back_buffer, (uint8_t *)front_buffer, WIDTH * HEIGHT * sizeof(Pixel));
        do_flip = false;
    }

    if(dma_channel_get_irq0_status(dma_channel)) {
        dma_channel_acknowledge_irq0(dma_channel);

        // SM is finished when it stalls on empty TX FIFO (or, y'know, DMA callback)
        //hub75_wait_tx_stall(pio, sm_data);

        // Check that previous OEn pulse is finished, else things WILL get out of sequence
        hub75_wait_tx_stall(pio, sm_row);

        // Latch row data, pulse output enable for new row.
        pio_sm_put_blocking(pio, sm_row, row | (3u * (1u << bit) << 5));

        row++;

        if(row == 32) {
            row = 0;
            bit++;
            if (bit == 12) {
                bit = 0;
            }
            hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, bit);
        }
    }

    dma_channel_set_trans_count(dma_channel, WIDTH * 4, false);
    dma_channel_set_read_addr(dma_channel, &back_buffer[row * WIDTH * 4], true);
}

void hub75_start() {
    // Needed for 64x64 register write
    setup_pin(DATA_BASE_PIN + 0);
    setup_pin(DATA_BASE_PIN + 1);
    setup_pin(DATA_BASE_PIN + 2);
    setup_pin(DATA_BASE_PIN + 3);
    setup_pin(DATA_BASE_PIN + 4);
    setup_pin(DATA_BASE_PIN + 5);
    setup_pin(CLK_PIN);
    setup_pin(STROBE_PIN);
    setup_pin(OEN_PIN);

    // Ridiculous register write nonsense for the FM6126A-based 64x64 matrix
    FM6126A_write_register(0b1111111111111110, 12);
    FM6126A_write_register(0b0000001000000000, 13);

    data_prog_offs = pio_add_program(pio, &hub75_data_rgb888_program);
    row_prog_offs = pio_add_program(pio, &hub75_row_program);
    hub75_data_rgb888_program_init(pio, sm_data, data_prog_offs, DATA_BASE_PIN, CLK_PIN);
    hub75_row_program_init(pio, sm_row, row_prog_offs, ROWSEL_BASE_PIN, ROWSEL_N_PINS, STROBE_PIN);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_bswap(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm_data, true));
    dma_channel_configure(dma_channel, &config, &pio->txf[sm_data], NULL, 0, false);
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_enabled(pio_get_dreq(pio, sm_data, true), true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete);
    irq_set_enabled(DMA_IRQ_0, true);

    row = 0;
    bit = 0;
    dma_complete();
}

void core1_main() {
    // Needed for 64x64 register write
    setup_pin(DATA_BASE_PIN + 0);
    setup_pin(DATA_BASE_PIN + 1);
    setup_pin(DATA_BASE_PIN + 2);
    setup_pin(DATA_BASE_PIN + 3);
    setup_pin(DATA_BASE_PIN + 4);
    setup_pin(DATA_BASE_PIN + 5);
    setup_pin(CLK_PIN);
    setup_pin(STROBE_PIN);
    setup_pin(OEN_PIN);

    // Ridiculous register write nonsense for the FM6126A-based 64x64 matrix
    FM6126A_write_register(0b1111111111111110, 12);
    FM6126A_write_register(0b0000001000000000, 13);

    data_prog_offs = pio_add_program(pio, &hub75_data_rgb888_program);
    row_prog_offs = pio_add_program(pio, &hub75_row_program);
    hub75_data_rgb888_program_init(pio, sm_data, data_prog_offs, DATA_BASE_PIN, CLK_PIN);
    hub75_row_program_init(pio, sm_row, row_prog_offs, ROWSEL_BASE_PIN, ROWSEL_N_PINS, STROBE_PIN);

    while (1) {
        if (do_flip) {
            memcpy((uint8_t *)back_buffer, (uint8_t *)front_buffer, WIDTH * HEIGHT * sizeof(Pixel));
            do_flip = false;
        }

        for (int bit = 0; bit < 12; ++bit) {
            uint32_t *data = &back_buffer[0];
            for (int rowsel = 0; rowsel < (1 << ROWSEL_N_PINS); ++rowsel) {
                hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, bit);
                for (int x = 0; x < WIDTH * 4; ++x) {
                    pio_sm_put_blocking(pio, sm_data, *data++);
                }

                // SM is finished when it stalls on empty TX FIFO
                hub75_wait_tx_stall(pio, sm_data);
                // Also check that previous OEn pulse is finished, else things can get out of sequence
                hub75_wait_tx_stall(pio, sm_row);

                // Latch row data, pulse output enable for new row.
                pio_sm_put_blocking(pio, sm_row, rowsel | (3u * (1u << bit) << 5));
            }
        }
    }

}

int main() {
    stdio_init_all();

    // Launch the display update routine on Core 1, it's hungry for cycles!
    //multicore_launch_core1(core1_main);
    hub75_start();

    // Basic loop to draw something to the screen.
    // This gets the distance from the middle of the display and uses it to paint a circular colour cycle.
    while (true) {
        float offset = millis() / 5000.0f;
        for(auto x = 0u; x < WIDTH; x++) {
            for(auto y = 0u; y < HEIGHT; y++) {
                // Center our rainbow circles
                float x1 = ((int)x - WIDTH / 2);
                float y1 = ((int)y - HEIGHT / 2);
                // Get hue as the distance from the display center as float from 0.0 to 1.0f.
                float h = float(x1*x1 + y1*y1) / float(WIDTH*WIDTH + HEIGHT*HEIGHT);
                // Offset our hue to animate the effect
                h -= offset;
                set_hsv(x, y, h, 1.0f, 1.0f);
            }

            // Monochrome gradient to verify gamma
            for(auto y = 0u; y < 10; y++) {
                set_rgb(x, y, x * 4, x * 4, x * 4);
            }
            set_rgb(x, 10, 0, 0, 0);

            // X cross to verify pixel alignment
            set_rgb(x, x, 255, 0, 0);
            set_rgb(x, WIDTH - 1 - x, 255, 0, 0);
        }

        flip();
        sleep_ms(1000 / 60);
    }
}
/*
 * Copyright (c) 2021-2022 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* MPR121 autoconfig_sensor example.

In addition to the MPR121 touch sensor, this example requires some
other components: an external button (BTN) wired as shown to a GPIO
(and debounced with a capacitor, C), and, optionally, two LEDs, each
connected to one GPIO:

GPIO
 |
 +---- C
 |     |
BTN    |
 |    GND
GND

GPIO - R - LED - GND

One touch sensor (electrode) is enabled in the MPR121. Then, just like
with the example `sensor_values`, this electrode is polled at 100-ms
intervals and three relevant values are printed: `is_touched`,
`baseline`, and `filtered`. The novelty in this example is that
pressing the button autoconfigures the electrode. If autoconfiguration
fails, one external LED lights up, and if it succeeds the other
external LED lights up (useful for debugging but these LEDs are
optional).

This is useful to investigate (and tune) the behaviour of the touch
sensor and it's ability to "adapt" (by triggering autoconfiguration) to
changing conditions in the sensor.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpr121.h"

#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

// MPR121 I2C definitions
#define MPR121_ADDR 0x5A
#define MPR121_I2C_FREQ 100000
#define MPR121_TOUCH_THRESHOLD 12
#define MPR121_RELEASE_THRESHOLD 6
struct mpr121_sensor mpr121;

// The Pico's onboard LED.
const uint8_t led = PICO_DEFAULT_LED_PIN;

// In this example two external LEDs are wired: one red to indicate
// autoconfiguration failure, and one green to indicate success. These
// LEDs are optional. The pins that control these LEDs are:
const uint8_t red_led = 14;
const uint8_t green_led = 15;

// An external button is connected to this pin.
const uint8_t button = 13;

// A callback triggered by the pressing of the button. This will
// autoconfigure the touch sensor, and light up a LED if it succeeds,
// or a different one if it fails.
void button_callback(uint gpio, uint32_t events) {
    bool autoconfig_successful = mpr121_autoconfig(&mpr121);
    if (autoconfig_successful) {
        gpio_put(green_led, 1);
        busy_wait_ms(150);
        gpio_put(green_led, 0);
    } else {
        gpio_put(red_led, 1);
        busy_wait_ms(150);
        gpio_put(red_led, 0);
    }
}

int main()
{
    stdio_init_all();

    // Initialise LEDs.
    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);
    gpio_init(red_led);
    gpio_set_dir(red_led, GPIO_OUT);
    gpio_init(green_led);
    gpio_set_dir(green_led, GPIO_OUT);

    // Initialise button pin. The callback function will be called when the button is pressed, thus falling to ground.
    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);
    gpio_set_irq_enabled_with_callback(button, GPIO_IRQ_EDGE_FALL,
        true, &button_callback);

    // I2C Initialisation. 
    i2c_init(I2C_PORT, MPR121_I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Touch sensor initialisation
    mpr121_init(I2C_PORT, MPR121_ADDR, &mpr121);
    mpr121_set_thresholds(MPR121_TOUCH_THRESHOLD,
                          MPR121_RELEASE_THRESHOLD, &mpr121);
    mpr121_autoconfig(&mpr121);
    // Enable only n=1 electrode (electrode 0).
    mpr121_enable_electrodes(1, &mpr121);

    const uint8_t electrode = 0;
    bool is_touched;
    uint16_t baseline, filtered;

    while(1) {
        mpr121_is_touched(electrode, &is_touched, &mpr121);
        mpr121_filtered_data(electrode, &filtered, &mpr121);
        mpr121_baseline_value(electrode, &baseline, &mpr121);
        if (is_touched) {
            gpio_put(led, 1);
        } else {
            gpio_put(led, 0);
        }
        printf("%d %d %d\n", baseline, filtered, is_touched);
        sleep_ms(100);
    }

    return 0;
}

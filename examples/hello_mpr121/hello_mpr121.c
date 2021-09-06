/*
 * Copyright (c) 2021 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* MPR121 hello_mpr121 example.

Whenever a sensor (electrode) is touched, the on-board LED lights on and the sensor number is printed.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C definitions: port and pin numbers
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// I2C definitions: address and frequency. These are the default
// values. Uncomment only if you need to re-define these.
// #define MPR121_ADDR 0x5A
// #define MPR121_I2C_FREQ 100000

// Touch and release thresholds. Uncomment to change these default
// values.
// #define MPR121_TOUCH_THRESHOLD 16
// #define MPR121_RELEASE_THRESHOLD 10

// Include the MPR11 library only after the definitions above.
#include "mpr121.h"

int main()
{
    stdio_init_all();

    // Setup the on-board LED.
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialise I2C.
    i2c_init(I2C_PORT, MPR121_I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Initialise and autoconfigure the touch sensor.
    mpr121_init();
    mpr121_autoconfig();

    // Initialise data variables.
    uint16_t touched;

    while (1)
    {
        // Bits 11-0 in the value returned by this function are flags
        // set when the electrodes are touched.
        mpr121_touched(&touched);

        // If a touch is detected, light on the LED and print a pattern
        // representing the electrodes (0 to 11) where 1 means touched.
        if (touched > 0) {
            gpio_put(LED_PIN, 1);
            for (uint8_t electrode = 0; electrode < 12; electrode++) {
                printf("%c ", ((touched >> electrode) & 1) ? '1' : '.');
            }
            printf("\n");
        } else {
            gpio_put(LED_PIN, 0);
        }

        // Pause for these many ms.
        sleep_ms(200);
    }

    return 0;
}

/*
 * Copyright (c) 2021 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/* MPR121 sensor_values example.

In this example only one touch sensor (electrode) is enabled. Then, this
electrode is polled at 100-ms intervals and

1. If the sensor has been touched, the on-board LED lights up.

2. Three values are read from that sensor and printed: (i) whether the
sensor has been touched (`is_touched`, bool), (ii) The baseline sensor
value (`baseline`, uint8), and (iii) the filtered value of the sensor
(`filtered`, uint8). It is possible to plot these values in real time
(e.g. with MicroDAQ, https://github.com/antgon/microdaq) to visualise,
debug, and tune the touch sensor.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpr121.h"

// I2C definitions: port and pin numbers
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// I2C definitions: address and frequency.
#define MPR121_ADDR 0x5A
#define MPR121_I2C_FREQ 100000

// Touch and release thresholds.
#define MPR121_TOUCH_THRESHOLD 16
#define MPR121_RELEASE_THRESHOLD 10

int main()
{
    stdio_init_all();

    // Setup the default, on-board LED.
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
    struct mpr121_sensor mpr121;
    mpr121_init(I2C_PORT, MPR121_ADDR, &mpr121);
    mpr121_set_thresholds(MPR121_TOUCH_THRESHOLD,
                          MPR121_RELEASE_THRESHOLD, &mpr121);
    mpr121_autoconfig(&mpr121);

    // Enable only one touch sensor (electrode 0).
    mpr121_enable_electrodes(1, &mpr121);

    // Initialise data variables.
    const uint8_t electrode = 0;
    bool is_touched;
    uint16_t baseline, filtered;

    while(1) {
        // Check if the electrode has been touched, and read the
        // baseline and filtered data values (useful for debugging and
        // tuning the sensor).
        mpr121_is_touched(electrode, &is_touched, &mpr121);
        mpr121_baseline_value(electrode, &baseline, &mpr121);
        mpr121_filtered_data(electrode, &filtered, &mpr121);

        // Switch on the on-board LED if the electrode has been touched.
        if (is_touched){
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        // Print the data.
        printf("%d %d %d\n", baseline, filtered, is_touched);

        // Pause for 100 ms.
        sleep_ms(100);
    }

    return 0;
}

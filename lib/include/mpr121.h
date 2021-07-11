/*
 * Copyright (c) 2021 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MPR121_H_
#define _MPR121_H_

#include "hardware/i2c.h"

#ifndef I2C_PORT
#define I2C_PORT i2c0
#endif

#ifndef MPR121_ADDR
#define MPR121_ADDR 0x5A
#endif

#ifndef MPR121_I2C_FREQ
#define MPR121_I2C_FREQ 100000
#endif

#ifndef MPR121_DEFAULT_TOUCH_THRESHOLD
#define MPR121_DEFAULT_TOUCH_THRESHOLD 16
#endif

#ifndef MPR121_DEFAULT_RELEASE_THRESHOLD
#define MPR121_DEFAULT_RELEASE_THRESHOLD 10
#endif


// Register map

#define MPR121_TOUCH_STATUS _u(0x00)
#define MPR121_OOR_STATUS _u(0x02)
#define MPR121_ELECTRODE_FILTERED_DATA _u(0x04)
#define MPR121_BASELINE_VALUE _u(0x1E)
// Registers 0x2B ~ 0x7F are control and configuration registers
#define MPR121_MAX_HALF_DELTA_RISING _u(0x2B) // MHDR
#define MPR121_NOISE_HALF_DELTA_RISING _u(0x2C) // NHDR
#define MPR121_NOISE_COUNT_LIMIT_RISING _u(0x2D) // NCLR
#define MPR121_FILTER_DELAY_COUNT_RISING _u(0x2E) // FDLR
#define MPR121_MAX_HALF_DELTA_FALLING _u(0x2F) // MHDF
#define MPR121_NOISE_HALF_DELTA_FALLING _u(0x30) // NHDF
#define MPR121_NOISE_COUNT_LIMIT_FALLING _u(0x31) // NCLF
#define MPR121_FILTER_DELAY_COUNT_FALLING _u(0x32) // FDLF
#define MPR121_NOISE_HALF_DELTA_TOUCHED _u(0x33) // NHDT
#define MPR121_NOISE_COUNT_LIMIT_TOUCHED _u(0x34) // NCLT
#define MPR121_FILTER_DELAY_COUNT_TOUCHED _u(0x35) // FDLT
// (ELEPROX 0x36 .. 0x40)
#define MPR121_TOUCH_THRESHOLD _u(0x41)
#define MPR121_RELEASE_THRESHOLD _u(0x42)
// (ELEPROX 0x59 .. 0x5A)
#define MPR121_DEBOUNCE _u(0x5B)
#define MPR121_CONFIG1 _u(0x5C)
#define MPR121_CONFIG2 _u(0x5D)
#define MPR121_ELECTRODE_CONFIG _u(0x5E) // ECR
#define MPR121_ELECTRODE_CURRENT _u(0x5F)
// (0x6C..0x72: Charge time registers.)
// (0x73..0x7A: GPIO registers. Allow to use ELE11 ~ ELE4 as GPIOs or
// LED drivers when not used for touch sensing.)
#define MPR121_AUTOCONFIG_CONTROL_0 _u(0x7B)
#define MPR121_AUTOCONFIG_CONTROL_1 _u(0x7C)
#define MPR121_AUTOCONFIG_USL _u(0x7D)
#define MPR121_AUTOCONFIG_LSL _u(0x7E)
#define MPR121_AUTOCONFIG_TARGET _u(0x7F)
#define MPR121_SOFT_RESET _u(0x80)


static void mpr121_write(uint8_t reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, buf, 2, false);
}

static void mpr121_read(uint8_t reg, uint8_t *val) {
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPR121_ADDR, val, 1, false);
}

static void mpr121_read16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPR121_ADDR, buf, 2, false);
    *val = buf[1] << 8 | buf[0];
}

static void mpr121_set_thresholds(uint8_t touch, uint8_t release) {
    // Thresholds in the range 0~0xFF. In a typical touch detection
    // application, threshold is typically in the range 0x04~0x10. The
    // touch threshold is several counts larger than the release
    // threshold. This is to provide hysteresis and to prevent noise and
    // jitter.
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG, &config);
    if (config != 0){
        // Stop mode
        mpr121_write(MPR121_ELECTRODE_CONFIG, 0x00);
    }
    
    for (uint8_t i=0; i<12; i++) {
        mpr121_write(MPR121_TOUCH_THRESHOLD + i * 2, touch);
        mpr121_write(MPR121_RELEASE_THRESHOLD + i * 2, release);
    }

    if (config != 0){
        mpr121_write(MPR121_ELECTRODE_CONFIG, config);
    }
}

void mpr121_init(void);

static void mpr121_enable_electrodes(uint8_t nelec){
    // Enable only the number of electrodes specified, e.g. if `nelec`
    // is 3, only electrodes 0 to 2 will be enabled; if it is 6,
    // electrodes 0 to 5 will be enabled. "Enabling specific channels
    // will save the scan time and sensing field power spent on the
    // unused channels."
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG, &config);
    config &= 0xf0 + nelec;
    mpr121_write(MPR121_ELECTRODE_CONFIG, 0x00);
    mpr121_write(MPR121_ELECTRODE_CONFIG, config);
}

static void mpr121_touched(uint16_t *val) {
    mpr121_read16(MPR121_TOUCH_STATUS, val);
    *val &= 0x0fff;
}

static void mpr121_is_touched(uint8_t electrode, bool *val){
    uint16_t touched;
    mpr121_touched(&touched);
    *val = (bool) (touched >> electrode) & 1;
}

static void mpr121_filtered_data(uint8_t electrode, uint16_t *val){
    mpr121_read16(MPR121_ELECTRODE_FILTERED_DATA + (electrode * 2), val);
    // Filtered data is 10-bit
    *val &= 0x3ff;
}

static void mpr121_baseline_value(uint8_t electrode, uint16_t *val){
    uint8_t baseline;
    mpr121_read(MPR121_BASELINE_VALUE + electrode, &baseline);
    // From the datasheet: Although internally the baseline value is
    // 10-bit, users can only access the 8 MSB of the 10-bit baseline
    // value through the baseline value registers. The read out from the
    // baseline register must be left shift two bits before comparing it
    // with the 10-bit electrode data.
    *val = baseline << 2;
}

/*! \brief Enable autoconfiguration
 *
 * Automatically configure charge current and charge time. The optimal
 * values used for this depend on Vdd, see NXP Application Note AN3889.
 * Here, these values have been calculated for Vdd = 3.3 V, which is
 * that in the Pico.
 */
void mpr121_set_autoconfig(void){};

#endif

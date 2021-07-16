/*
 * Copyright (c) 2021 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MPR121_H_
#define _MPR121_H_

#include "pico.h"
#include "hardware/i2c.h"

/** \file mpr121.h
 * \brief Library for using an MPR121-based touch sensor with the Raspberry Pi
 * Pico
 *
*/

#ifndef I2C_PORT
#define I2C_PORT i2c0
#endif

#ifndef MPR121_ADDR
#define MPR121_ADDR 0x5A
#endif

#ifndef MPR121_I2C_FREQ
#define MPR121_I2C_FREQ 100000
#endif

#ifndef MPR121_TOUCH_THRESHOLD
#define MPR121_TOUCH_THRESHOLD 16
#endif

#ifndef MPR121_RELEASE_THRESHOLD
#define MPR121_RELEASE_THRESHOLD 10
#endif

/*! \brief MPR121 register map
 */
enum mpr121_register {
    MPR121_TOUCH_STATUS_REG = 0x00u,
    MPR121_OUT_OF_RANGE_STATUS_0_REG = 0x02u,
    MPR121_OUT_OF_RANGE_STATUS_1_REG = 0x03u,
    MPR121_ELECTRODE_FILTERED_DATA_REG = 0x04u,
    MPR121_BASELINE_VALUE_REG = 0x1Eu,
    // Registers 0x2B ~ 0x7F are control and configuration registers
    MPR121_MAX_HALF_DELTA_RISING_REG = 0x2Bu,
    MPR121_NOISE_HALF_DELTA_RISING_REG = 0x2Cu,
    MPR121_NOISE_COUNT_LIMIT_RISING_REG = 0x2Du,
    MPR121_FILTER_DELAY_COUNT_RISING_REG = 0x2Eu,
    MPR121_MAX_HALF_DELTA_FALLING_REG = 0x2Fu,
    MPR121_NOISE_HALF_DELTA_FALLING_REG = 0x30u,
    MPR121_NOISE_COUNT_LIMIT_FALLING_REG = 0x31u,
    MPR121_FILTER_DELAY_COUNT_FALLING_REG = 0x32u,
    MPR121_NOISE_HALF_DELTA_TOUCHED_REG = 0x33u,
    MPR121_NOISE_COUNT_LIMIT_TOUCHED_REG = 0x34u,
    MPR121_FILTER_DELAY_COUNT_TOUCHED_REG = 0x35u,
    // (ELEPROX 0x36 .. 0x40)
    MPR121_TOUCH_THRESHOLD_REG = 0x41u,
    MPR121_RELEASE_THRESHOLD_REG = 0x42u,
    // (ELEPROX 0x59 .. 0x5A)
    MPR121_DEBOUNCE_REG = 0x5Bu,
    MPR121_AFE_CONFIG_REG = 0x5Cu,
    MPR121_FILTER_CONFIG_REG = 0x5Du,
    MPR121_ELECTRODE_CONFIG_REG = 0x5Eu,
    MPR121_ELECTRODE_CURRENT_REG = 0x5Fu,
    // (0x6C..0x72: Charge time registers.)
    // (0x73..0x7A: GPIO registers. Allow to use ELE11 ~ ELE4 as GPIOs or
    // LED drivers when not used for touch sensing.)
    MPR121_AUTOCONFIG_CONTROL_0_REG = 0x7Bu,
    MPR121_AUTOCONFIG_CONTROL_1_REG = 0x7Cu,
    MPR121_AUTOCONFIG_USL_REG = 0x7Du,
    MPR121_AUTOCONFIG_LSL_REG = 0x7Eu,
    MPR121_AUTOCONFIG_TARGET_REG = 0x7Fu,
    MPR121_SOFT_RESET_REG = 0x80u
};

/*! \brief Resets the MPR121 and configures registers
 *
 */
void mpr121_init(void);

/*! \brief Autoconfigure sensors
 *
 * Automatically configure charge current and charge time. The optimal
 * values used for this depend on Vdd, see NXP Application Note AN3889.
 * Here, these values have been calculated for Vdd = 3.3 V, which is
 * that in the Pico.
 * 
 * \return true if autoconfiguration was successful
 */
bool mpr121_autoconfig(void);

/*! \brief Write a value to the specified register
 *
 * \param reg The register address
 * \param val The value to write
 */
static void mpr121_write(enum mpr121_register reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, buf, 2, false);
}

/*! \brief Read a byte from the specified register
 *
 * \param reg The register address
 * \param val The value to read into
 */
static void mpr121_read(enum mpr121_register reg, uint8_t *val) {
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPR121_ADDR, val, 1, false);
}

/*! \brief Read a 2-byte value from the specified register
 *
 * \param reg The register address
 * \param val The value to read into
 */
static void mpr121_read16(enum mpr121_register reg, uint16_t *val) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPR121_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPR121_ADDR, buf, 2, false);
    *val = buf[1] << 8 | buf[0];
}

/*! \brief Set touch and release thresholds
 * 
 * In a typical application, touch threshold is in the range 4..16,
 * and it is several counts larger than the release threshold. This
 * is to provide hysteresis and to prevent noise and jitter.
 *  
 * \param touch Touch threshold in the range 0..255
 * \param release Release threshold in the range 0..255
 */
static void mpr121_set_thresholds(uint8_t touch, uint8_t release) {
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG_REG, &config);
    if (config != 0){
        // Stop mode
        mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00);
    }
    
    for (uint8_t i=0; i<12; i++) {
        mpr121_write(MPR121_TOUCH_THRESHOLD_REG + i * 2, touch);
        mpr121_write(MPR121_RELEASE_THRESHOLD_REG + i * 2, release);
    }

    if (config != 0){
        mpr121_write(MPR121_ELECTRODE_CONFIG_REG, config);
    }
}

/*! \brief Enable only the number of electrodes specified
 * 
 * E.g. if `nelec` is 3, only electrodes 0 to 2 will be enabled; if it
 * is 6, electrodes 0 to 5 will be enabled. From the datasheet:
 * "Enabling specific channels will save the scan time and sensing
 * field power spent on the unused channels."
 * 
 * \param nelec Number of electrodes to enable
*/
static void mpr121_enable_electrodes(uint8_t nelec){
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG_REG, &config);
    config &= 0xf0 + nelec;
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00);
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, config);
}

static void mpr121_touched(uint16_t *val) {
    mpr121_read16(MPR121_TOUCH_STATUS_REG, val);
    *val &= 0x0fff;
}

static void mpr121_is_touched(uint8_t electrode, bool *val){
    uint16_t touched;
    mpr121_touched(&touched);
    *val = (bool) (touched >> electrode) & 1;
}

static void mpr121_filtered_data(uint8_t electrode, uint16_t *val){
    mpr121_read16(MPR121_ELECTRODE_FILTERED_DATA_REG + (electrode * 2), val);
    // Filtered data is 10-bit
    *val &= 0x3ff;
}

/*! \brief Read the baseline value from an electrode
 *
 * \param electrode The electrode number to read the value from
 * \param val The value to read into
 */
static void mpr121_baseline_value(uint8_t electrode, uint16_t *val){
    uint8_t baseline;
    mpr121_read(MPR121_BASELINE_VALUE_REG + electrode, &baseline);
    // From the datasheet: Although internally the baseline value is
    // 10-bit, users can only access the 8 MSB of the 10-bit baseline
    // value through the baseline value registers. The read out from the
    // baseline register must be left shift two bits before comparing it
    // with the 10-bit electrode data.
    *val = baseline << 2;
}

#endif

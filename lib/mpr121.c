/*
 * Copyright (c) 2021 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mpr121.h"

void mpr121_init(void) {
    // Resets the MPR121 and configures registers.

    // Writing 0x80 (SOFT_RESET) with 0x63 asserts soft reset.
    mpr121_write(MPR121_SOFT_RESET_REG, 0x63);
    
    // Enter stop mode: set all ELEPROX_EN and ELE_EN bits to zero. This
    // is because register write operations can only be done in stop
    // mode.
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00);

    // Set touch and release thresholds.
    mpr121_set_thresholds(MPR121_DEFAULT_TOUCH_THRESHOLD,
                          MPR121_DEFAULT_RELEASE_THRESHOLD);

    // Configure electrode filtered data and baseline registers

    // Maximum Half Delta (MHD): Determines the largest magnitude of
    // variation to pass through the baseline filter. The range of the
    // effective value is 1~63.
    // Initial values: rising, 0x2B; falling, 0x2F.
    mpr121_write(MPR121_MAX_HALF_DELTA_RISING_REG, 0x01);
    mpr121_write(MPR121_MAX_HALF_DELTA_FALLING_REG, 0x01);
    
    // Noise Half Delta (NHD): Determines the incremental change when
    // non-noise drift is detected. The range of the effective value is
    // 1~63.
    // Initial values: rising, 0x2C; falling, 0x30; touched, 0x33.
    mpr121_write(MPR121_NOISE_HALF_DELTA_RISING_REG, 0x01);
    mpr121_write(MPR121_NOISE_HALF_DELTA_FALLING_REG, 0x05);
    mpr121_write(MPR121_NOISE_HALF_DELTA_TOUCHED_REG, 0x00);
    
    // Noise Count Limit (NCL): Determines the number of samples
    // consecutively greater than the Max Half Delta value. This is
    // necessary to determine that it is not noise. The range of the
    // effective value is 0~255.
    // Initial values: rising, 0x2D; falling, 0x31; touched, 0x34.
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_RISING_REG, 0x0E);
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_FALLING_REG, 0x01);
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_TOUCHED_REG, 0x00);
    
    // Filter Delay Count Limit (FDL): Determines the operation rate of
    // the filter. A larger count limit means the filter delay is
    // operating more slowly. The range of the effective value is 0~255.
    // Initial values: rising, 0x2E; falling, 0x32; touched, 0x35.
    mpr121_write(MPR121_FILTER_DELAY_COUNT_RISING_REG, 0x00);
    mpr121_write(MPR121_FILTER_DELAY_COUNT_FALLING_REG, 0x00);
    mpr121_write(MPR121_FILTER_DELAY_COUNT_TOUCHED_REG, 0x00);

    // Debounce. Value range for each is 0~7.
    // Bits 2-0, debounce touch (DT).
    // Bits 6-4, debounce release (DR).
    // These affect the status bits in the status register: the status
    // bits will only take place after the number of consecutive touch
    // or release detection meets the debounce number setting. The
    // status bit change will have a delay if debounce is set.
    mpr121_write(MPR121_DEBOUNCE_REG, 0x00);

    // Global filter and charge/discharge settings

    // Register 0x5C (AFE configuration register)
    //
    // First filter iterations (FFI), bits 7-6. Number of samples taken
    // as input to the first level of filtering. Default is 0b00 = sets
    // samples taken to 6.
    //
    // Charge-discharge current (CDC), bits 5-0. Sets the value of
    // charge-discharge current. Max is 63 µA in 1 µA steps. Default is
    // 16 (0b010000) = 16 µA.
    //
    // Defaults = 0b00010000 = 0x10
    mpr121_write(MPR121_AFE_CONFIG_REG , 0x10);

    // Register 0x5D (Filter configuration register)
    //
    // Charge discharge time (CDT), bits 7-5. Selects the global value
    // of charge time applied to electrode. The maximum is 32 μs,
    // programmable as 2 ^(n-2) μs. Default is 0b001 = time set to 0.5
    // µs.
    //
    // Second filter iterations (SFI), bits 4-3. Selects the number of
    // samples taken for the second level filter. Default is 0b00 =
    // number of samples is set to 4.
    //
    // Electrode sample interval (ESI), bits 2-0. Selects the period
    // between samples used for the second level of filtering. The
    // maximum is 128 ms, programmable to 2^n ms. Default is 0b100
    // (encoding 4) = 16 ms.
    // 
    // Defaults = 0b00100100 = 0x24
    mpr121_write(MPR121_FILTER_CONFIG_REG, 0x24);

    // Electrode configuration (register 0x5E).
    //
    // Determines if the MPR121 is in Run Mode or Stop Mode, controls the
    // baseline tracking operation and specifies the input
    // configurations of the 13 channels.

    // Enter run mode: enable all electrodes. The electrode
    // configuration register controls these:
    //
    // Calibration lock (CL), bits 7-6. Baseline tracking is to
    // compensate for the environment and background induced slow
    // capacitance change to the input sensing channels. Enabled by
    // default (value 0b00). Value 0b11 enables tracking and also loads
    // initial value from electrode data.
    //
    // Proximity enable (ELEPROX_EN), bits 5-4. Default = 0b00
    // (disabled).
    //
    // Electrode enable (ELE_EN), bits 3-0. Sets Run Mode and controls
    // the operation of the 12 electrodes. Default = 0b1111 = Run Mode
    // for all 12 electrodes.
    //
    // Value 0b00001111 = 0x0F Value 0b11001111 = 0xCF
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0xCF);
}

bool mpr121_autoconfig(void){
    // Read current configuration and then enter stop mode. Stop mode
    // is needed because register write operations can only be done in
    // this mode.
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG_REG, &config);
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00);

    // Autoconfig USL register: the upper limit for the
    // auto-configuration. This value (and those that follow below)
    // were calculated based on Vdd = 3.3 V and following the equations
    // in NXP Application Note AN3889.
    // USL = 201 = 0xC9
    mpr121_write(MPR121_AUTOCONFIG_USL_REG, 0xC9);

    // Autoconfig target level register: the target level for the 
    // auto-configuration baseline search.
    // TL = 181 = 0xB5
    mpr121_write(MPR121_AUTOCONFIG_TARGET_REG, 0xB5);

    // Autoconfig LSL register: the lower limit for the
    // auto-configuration.
    // LSL = 131 = 0x83
    mpr121_write(MPR121_AUTOCONFIG_LSL_REG, 0x83);

    // Autoconfiguration control register.
    //
    // Default value is 0b00001011 = 0x0B, where:
    //
    // First filter iterations (FFI), bits 7-6. Must be the same value
    // of FFI as in the general configuration, see mpr121_init().
    // Default is 0b00.
    //
    // Retry, bits 5-4. Default is diabled, 0b00.
    //
    // Baseline value adjust (BVA), bits 3-2. Default is 0b10, which
    // allows the baseline to be adjusted.
    //
    // Automatic Reconfiguration Enable (ARE), bit 1. Default is 0b1,
    // enabled.
    //
    // Automatic Reconfiguration Enable (ACE), bit 0. Default is 0b1,
    // enabled.
    mpr121_write(MPR121_AUTOCONFIG_CONTROL_0_REG, 0x0B);

    // Re-enable electrode(s) by writing back the configuration bits.
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, config);

    // Check to see if autoconfiguration worked. Bits 7-6 in ELE-8
    // out-of-range register are flags that are set when
    // auto-(re)configuration fails.
    bool success = false;
    uint8_t out_of_range_status;
    mpr121_read(MPR121_OUT_OF_RANGE_STATUS_1_REG, &out_of_range_status);
    if ((out_of_range_status & 0b11000000) == 0){
        success = true;
    }
    return success;
}
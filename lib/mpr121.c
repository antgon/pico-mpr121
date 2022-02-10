/*
 * Copyright (c) 2021-2022 Antonio González
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mpr121.h"

void mpr121_init(i2c_inst_t *i2c_port, uint8_t i2c_addr,
                 mpr121_sensor_t *sensor) {
    sensor->i2c_port = i2c_port;
    sensor->i2c_addr = i2c_addr;

    // Writing 0x80 (SOFT_RESET) with 0x63 asserts soft reset.
    mpr121_write(MPR121_SOFT_RESET_REG, 0x63, sensor);
    
    // Enter stop mode: set all ELEPROX_EN and ELE_EN bits to zero. This
    // is because register write operations can only be done in stop
    // mode.
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00, sensor);

    // Configure electrode filtered data and baseline registers

    // Maximum Half Delta (MHD): Determines the largest magnitude of
    // variation to pass through the baseline filter. The range of the
    // effective value is 1~63.
    mpr121_write(MPR121_MAX_HALF_DELTA_RISING_REG, 0x01, sensor);
    mpr121_write(MPR121_MAX_HALF_DELTA_FALLING_REG, 0x01, sensor);
    
    // Noise Half Delta (NHD): Determines the incremental change when
    // non-noise drift is detected. The range of the effective value is
    // 1~63.
    mpr121_write(MPR121_NOISE_HALF_DELTA_RISING_REG, 0x01, sensor);
    mpr121_write(MPR121_NOISE_HALF_DELTA_FALLING_REG, 0x01, sensor);
    mpr121_write(MPR121_NOISE_HALF_DELTA_TOUCHED_REG, 0x00, sensor);  
    
    // Noise Count Limit (NCL): Determines the number of samples
    // consecutively greater than the Max Half Delta value. This is
    // necessary to determine that it is not noise. The range of the
    // effective value is 0~255.
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_RISING_REG, 0x00, sensor);
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_FALLING_REG, 0xff, sensor);
    mpr121_write(MPR121_NOISE_COUNT_LIMIT_TOUCHED_REG, 0x00, sensor);
    
    // Filter Delay Count Limit (FDL): Determines the operation rate of
    // the filter. A larger count limit means the filter delay is
    // operating more slowly. The range of the effective value is 0~255.
    mpr121_write(MPR121_FILTER_DELAY_COUNT_RISING_REG, 0x00, sensor);
    mpr121_write(MPR121_FILTER_DELAY_COUNT_FALLING_REG, 0x02, sensor);
    mpr121_write(MPR121_FILTER_DELAY_COUNT_TOUCHED_REG, 0x00, sensor);

    // Debounce. Value range for each is 0~7.
    // Bits 2-0, debounce touch (DT).
    // Bits 6-4, debounce release (DR).
    // These affect the status bits in the status register: the status
    // bits will only take place after the number of consecutive touch
    // or release detection meets the debounce number setting. The
    // status bit change will have a delay if debounce is set.
    mpr121_write(MPR121_DEBOUNCE_REG, 0x00, sensor);

    // Global filter and charge/discharge settings

    // Register 0x5C: Filter/global CDC configuration register
    //
    // First filter iterations (FFI), bits 7-6. Number of samples taken
    // as input to the first level of filtering.
    //     00 - Sets samples taken to 6 (default)
    //     01 - Sets samples taken to 10
    //     10 - Sets samples taken to 18
    //     11 - Sets samples taken to 34
    //
    // Charge-discharge current (CDC), bits 5-0. Sets the value of
    // charge-discharge current applied to the electrode. Max is 63 µA
    // in 1 µA steps. Default is
    //     000000 - Disable electrode charging
    //     000001 - Sets the current to 1 µA
    //     ~ 
    //     010000 - Sets the current to 16 µA (default)
    //     ~
    //     111111 - Sets the current to 63 µA
    //
    // AFE configuration register default = 0b00010000 = 0x10
    mpr121_write(MPR121_AFE_CONFIG_REG , 0x10, sensor);

    // Register 0x5D: Filter/global CDC configuration register
    //
    // Charge discharge time (CDT), bits 7-5. Selects the global value
    // of charge time applied to electrode. The maximum is 32 μs,
    // programmable as 2^(n-2) μs.
    //     000 - Disables electrode charging
    //     001 - Time is set to 0.5 µs (default)
    //     010 - Time is set to 1 µs
    //     ~
    //     111 - Time is set to 32 µs
    //
    // Second filter iterations (SFI), bits 4-3. Selects the number of
    // samples taken for the second level filter.
    //     00 - Number of samples is set to 4 (default)
    //     01 - Number of samples is set to 6
    //     10 - Number of samples is set to 10
    //     11 - Number of samples is set to 18
    //
    // Electrode sample interval (ESI), bits 2-0. Controls the sampling
    // rate of the device. The maximum is 128 ms, programmable to 2^n
    // ms.  Decrease this value for better response time, increase to
    // save power.
    //     000 - Period set to 1 ms
    //     001 - Period set to 2 ms
    //     ~
    //     100 - Period set to 16 ms (default)
    //     ~
    //     111 - Period set to 128 ms
    // 
    // Filter configuration register default = 0b00100100 = 0x24
    // Quick Start Guide (AN3944) suggests 0x04.
    mpr121_write(MPR121_FILTER_CONFIG_REG, 0x04, sensor);

    // Electrode configuration (register 0x5E).
    //
    // Determines if the MPR121 is in Run Mode or Stop Mode, controls
    // the baseline tracking operation and specifies the input
    // configurations of the 13 channels.

    // Enter run mode: enable all electrodes. The electrode
    // configuration register controls these:
    //
    // Calibration lock (CL), bits 7-6. Baseline tracking is to
    // compensate for the environment and background induced slow
    // capacitance change to the input sensing channels. Enabled by
    // default (value 0b00). Value 0b11 enables tracking and also loads
    // initial value from electrode data.
    //     00 - Baseline tracking enabled, initial baseline value is
    //          current value in baseline value register (default)
    //     01 - Baseline tracking disabled
    //     10 - Baseline tracking enabled, initial baseline value is
    //          loaded with the 5 high bits of the first 10-bit
    //          electrode data value
    //     11 - Baseline tracking enabled, initial baseline value is
    //          loaded with all 10 bits of the first electrode data
    //          value
    //
    // Proximity enable (ELEPROX_EN), bits 5-4. Default = 0b00
    // (disabled).
    //     00 - Proximity detection is disabled (default)
    //     01 - Run mMode with ELE0~ELE1 combined for proximity
    //          detection enabled
    //     10 - Run Mode with ELE0~ELE3 combined for proximity
    //          detection enabled
    //     11 - Run Mode with ELE0~ELE11 combined for proximity
    //          detection enabled
    //
    // Electrode enable (ELE_EN), bits 3-0. Sets Run Mode and controls
    // the operation of the 12 electrodes detection.
    //    0000 - Electrode detection is disabled (default)
    //    0001 - Run Mode with ELE0 for electrode detection enabled
    //    ~
    //    1011 - Run Mode with ELE0~ELE10 for electrode detection
    //           enabled
    //    11xx - Run Mode with ELE0~ELE11 for electrode detection
    //           enabled
    //
    // ELE_EN default value, 0x0C, which enables all 12 electrodes
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x0C, sensor);
}

bool mpr121_autoconfig(mpr121_sensor_t *sensor){
    // Read current configuration and then enter stop mode. Stop mode
    // is needed because register write operations can only be done in
    // this mode.
    uint8_t config;
    mpr121_read(MPR121_ELECTRODE_CONFIG_REG, &config, sensor);
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, 0x00, sensor);

    // Autoconfig USL register: the upper limit for the
    // auto-configuration. This value (and those that follow below)
    // were calculated based on Vdd = 3.3 V and following the equations
    // in NXP Application Note AN3889.
    // USL = 201 = 0xC9
    mpr121_write(MPR121_AUTOCONFIG_USL_REG, 0xC9, sensor);

    // Autoconfig target level register: the target level for the 
    // auto-configuration baseline search.
    // TL = 181 = 0xB5
    mpr121_write(MPR121_AUTOCONFIG_TARGET_REG, 0xB5, sensor);

    // Autoconfig LSL register: the lower limit for the
    // auto-configuration.
    // LSL = 131 = 0x83
    mpr121_write(MPR121_AUTOCONFIG_LSL_REG, 0x83, sensor);

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
    mpr121_write(MPR121_AUTOCONFIG_CONTROL_0_REG, 0x0B, sensor);

    // Re-enable electrode(s) by writing back the configuration bits.
    mpr121_write(MPR121_ELECTRODE_CONFIG_REG, config, sensor);

    // Check to see if autoconfiguration worked. Bits 7-6 in ELE-8
    // out-of-range register are flags that are set when
    // auto-(re)configuration fails.
    bool success = false;
    uint8_t out_of_range_status;
    mpr121_read(MPR121_OUT_OF_RANGE_STATUS_1_REG, &out_of_range_status,
                sensor);
    if ((out_of_range_status & 0b11000000) == 0){
        success = true;
    }
    return success;
}
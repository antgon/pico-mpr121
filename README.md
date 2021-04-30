# pico-mpr121

A C library for using an MPR121-based touch sensor with the Raspberry Pi Pico

The [MPR121](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf) is a capcitive touch sensor controller with 12 electrodes. It forms the basis of a number of touch sensor boards such as Adafruit [product 1982](https://www.adafruit.com/product/1982) and Seeed Studio [Grove I2C Touch Sensor](https://wiki.seeedstudio.com/Grove-I2C_Touch_Sensor/).


## Wiring

MPR121 | Pico 
-------|-----
SDA | SDA (e.g. pin 8)
SCL | SCL (e.g. pin 9)
VDD | 3V3(OUT)
GND | GND

## Software

1. Configure `CMakeLists.txt` in your base project to include the path to the **pico-mpr121** library. Add **pico-mpr121** to the list of target libraries. E.g. if the **pico-mpr121** library is located one directory above (`../`) the current one your `CMakeLists.txt` file should include
```cmake
.
.
.
include(pico_sdk_import.cmake)
add_subdirectory(../pico-mpr121/lib mpr121)

target_link_libraries(touch
        pico_stdlib
        hardware_i2c
        pico-mpr121
)
.
.
.
```

2. Use the library. See the examples directory for the full code.
```c
.
.
.
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#include "mpr121.h"

int main() {
    ...
}
```


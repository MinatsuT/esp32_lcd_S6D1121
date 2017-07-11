# ESP32 S6D1121 LCD Parallel Connection Driver
This is a S6D1121 LCD Driver for ESP32. Before compile this project, please install [ESP-IDF](https://github.com/espressif/esp-idf) and set `$(IDF_PATH)`:

`% export IDF_PATH=/your/esp-idf/path`

Then compile and flash by the following command:

`% make flash monitor`

# How to setup
Connect your ESP32 and LCD as follows:
<PRE>
ESP32   LCD
-----   ---
3V3  -- 3V3
GND  -- GND and LED
IO23 -- RS
IO22 -- WR
IO21 -- RD (Optional: If connect them, you can read ID of the LCD controller which is always '1121'.)
IO19 -- RST

IO18 -- D0
IO5  -- D1
IO17 -- D2
IO16 -- D3
IO4  -- D4
IO0  -- D5
IO2  -- D6
IO15 -- D7
</PRE>

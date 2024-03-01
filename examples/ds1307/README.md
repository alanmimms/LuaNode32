## DS1307

DS1307 is an RTC module. This example is modified from Kolban's example. 

## How to use

To use DS1307 module, please remove R2 and R3 on the module.

Pin connections:


--------
| DS1307 | ESP32 |
| --- | --- |
| SCL | GPIO19 |
| SDA | GPIO18 |
| Vcc | 3v3 |
| GND | GND |


Modified the SSID and password defined in `main.c`

Compile and run the example on ESP32, then you can see the time is output each second.

# Arduino library for AllSensors DLHR Series Low Voltage Digital Pressure Sensors #

This is an Arduino-compatible library for the [AllSensors DLHR Series Low Voltage Digital Pressure Sensors](https://www.allsensors.com/products/dlhr-series). The library currently supports I²C communications with the sensors.

This forked proposes the following additions:

* Add support for SPI (under development)
* Add a non blocking code function: readDataAsynchro(), to multiplex sensors.
    i2c multiplexing requires a multiplexer
    spi multiplexing is underdevelopment

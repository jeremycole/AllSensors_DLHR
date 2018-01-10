/*

Support for the AllSensors DLHR Series Low Voltage Digital Pressure Sensors.

See the following datasheet:

https://www.allsensors.com/datasheets/DS-0350_Rev_A.pdf

The sensors are sold in many varieties, with the following options:
  * Pressure range options from 0.5 to 60 inH2O
  * Type options of either differential or gage output.
  * Resolution options of 16-, 17-, or 18-bit.

For configuration simplicity, the pre-configured subclasses of AllSensors_DLHR which align
well with the purchaseable model numbers of the sensors can can be used instead of using
the AllSensors_DLHR class directly; see AllSensors_DLHR_subclasses.h.

* * *

This software is licensed under the Revised (3-clause) BSD license as follows:

Copyright (c) 2018, Jeremy Cole <jeremy@jcole.us>

All rights reserved.

See the LICENSE file for more details.

*/

#ifndef ALLSENSORS_DLHR_H
#define ALLSENSORS_DLHR_H

#include <stdint.h>

#include <Wire.h>

class AllSensors_DLHR {
public:

  // The default I2C address from the datasheet.
  static const uint8_t I2C_ADDRESS = 0x29;

  // The sensor type, where part numbers:
  //   * DLHR-xxxG-* are GAGE sensors.
  //   * DLHR-xxxD-* are DIFFERENTIAL sensors.
  enum SensorType {
    GAGE         = 'G',
    DIFFERENTIAL = 'D',
  };

  // The sensor's resolution for pressure readings, where part numbers:
  //   * DLHR-*-6 are 16-bit resolution.
  //   * DLHR-*-7 are 17-bit resolution.
  //   * DLHR-*-8 are 18-bit resolution.
  enum SensorResolution {
    RESOLUTION_16_BITS = 16,
    RESOLUTION_17_BITS = 17,
    RESOLUTION_18_BITS = 18,
  };

  enum MeasurementType {
    SINGLE    = 0xAA,
    AVERAGE2  = 0xAC,
    AVERAGE4  = 0xAD,
    AVERAGE8  = 0xAE,
    AVERAGE16 = 0xAF,
  };

  enum StatusFlags {
    RESERVED_7    = 1<<7,
    POWER_ON      = 1<<6,
    BUSY          = 1<<5,
    MODE          = 1<<4 | 1<<3,
    ERROR_MEMORY  = 1<<2,
    CONFIGURATION = 1<<1,
    ERROR_ALU     = 1<<0,
  };

  enum PressureUnit {
    IN_H2O = 'H',
    PASCAL = 'P',
  };

  enum TemperatureUnit {
    CELCIUS    = 'C',
    FAHRENHEIT = 'F',
    KELVIN     = 'K',
  };

private:

  // The length of the status information in the I2C response.
  static const uint8_t READ_STATUS_LENGTH = 1;

  // The length of the pressure data in the I2C response.
  static const uint8_t READ_PRESSURE_LENGTH = 3;

  // The length of the temperature data in the I2C response.
  static const uint8_t READ_TEMPERATURE_LENGTH = 3;

  // The "full scale" resolution of the pressure and temperature data, which is fixed
  // at 24 bits.
  static const uint8_t FULL_SCALE_RESOLUTION = 24;

  // The actual temperature resolution, which is fixed at 16 bits.
  static const uint8_t TEMPERATURE_RESOLUTION = 16;

  // The value of a full scale temperature or pressure, 2^24.
  static constexpr uint32_t FULL_SCALE_REF = (uint32_t) 1 << FULL_SCALE_RESOLUTION;

  TwoWire *bus;
  SensorType type;
  SensorResolution pressure_resolution;
  float pressure_max;
  float pressure_range;
  float pressure_zero_ref;
  PressureUnit pressure_unit;
  TemperatureUnit temperature_unit;

  // Convert a raw digital pressure read from the sensor to a floating point value in inH2O.
  float transferPressure(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Pressure(inH2O) = 1.25 x ((P_out_dig - OS_dig) / 2^24) x FSS(inH2O)
    return 1.25 * (((float) raw_value - pressure_zero_ref) / FULL_SCALE_REF) * pressure_range;    
  }

  // Convert a raw digital temperature read from the sensor to a floating point value in Celcius.
  float transferTemperature(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Temperature(degC) = ((T_out_dig * 125) / 2^24) - 40
    return (((float) raw_value * 125.0) / FULL_SCALE_REF) - 40.0;
  }

  // Convert the input in inH2O to the configured pressure output unit.
  float convertPressure(float in_h2o) {
    switch(pressure_unit) {
      case PASCAL:
        return 249.08 * in_h2o;
      case IN_H2O:
      default:
        return in_h2o;
    }
  }

  // Convert the input in Celcius to the configured temperature output unit.
  float convertTemperature(float degree_c) {
    switch(temperature_unit) {
      case FAHRENHEIT:
        return degree_c * 1.8 + 32.0;
      case KELVIN:
        return degree_c + 273.15;
      case CELCIUS:
      default:
        return degree_c;
    }    
  }

public:

  uint8_t status;
  float pressure;
  float temperature;

  uint32_t pressure_resolution_mask;
  uint32_t temperature_resolution_mask;

  uint32_t raw_p = 0;
  uint32_t raw_t = 0;

  // Return true if the status byte input indicates that the sensor is currently busy.
  static bool isBusy(uint8_t status_arg) {
    return (status_arg & StatusFlags::BUSY) != 0;
  }

  // Return true if the status byte input indicates that the sensor experienced an error.
  static bool isError(uint8_t status_arg) {
    return (status_arg & (StatusFlags::ERROR_MEMORY | StatusFlags::ERROR_ALU)) != 0;
  }

  AllSensors_DLHR(TwoWire *bus, SensorType type, SensorResolution pressure_resolution, float pressure_max);

  // Set the configured pressure unit for data output (the default is inH2O).
  void setPressureUnit(PressureUnit pressure_unit) {
    this->pressure_unit = pressure_unit;
  }

  // Set the configured temperature unit for data output (the default is Celcius).
  void setTemperatureUnit(TemperatureUnit temperature_unit) {
    this->temperature_unit = temperature_unit;
  }

  // Request that the sensor start a measurement. Following this command, the sensor will begin a
  // measurement which takes between 2.8ms (Single 16-bit) and 61.9ms (Average16 18-bit) to complete.
  // In the interim, isBusy() or (readStatus() directly) can be polled until the sensor no longer
  // reports itself busy, at which time the data registers can be read to find the result.
  //
  // If no argument is provided, a single measurement is taken. Otherwise, a MeasurementType can be
  // provided to allow multiple averaged measurements to be taken.
  void startMeasurement(MeasurementType measurement_type = MeasurementType::SINGLE);

  // Read the current status register from the sensor.
  uint8_t readStatus();

  // A simple wrapper around reading the current status register and returning true if it indicates
  // that the sensor is busy.
  bool isBusy() {
    return isBusy(readStatus());
  }

  // Read the data from the sensor. Passing a "wait" argument polls the sensor repeatedly until it is
  // no longer busy. If wait is false, if the sensor is busy return "true" immediately indicating that
  // data was not available. 
  bool readData(bool wait = true);
};

#include "AllSensors_DLHR_subclasses.h"

#endif // ALLSENSORS_DLHR_H

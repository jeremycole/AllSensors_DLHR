/*

This software is licensed under the Revised (3-clause) BSD license as follows:

Copyright (c) 2018, Jeremy Cole <jeremy@jcole.us>

All rights reserved.

See the LICENSE file for more details.

*/

#include "AllSensors_DLHR.h"

#include <math.h>
#include <util/delay.h>


AllSensors_DLHR::AllSensors_DLHR(TwoWire *bus, SensorType type, SensorResolution pressure_resolution, float pressure_max) :
  pressure_unit(PressureUnit::IN_H2O),
  temperature_unit(TemperatureUnit::CELCIUS)
{
  this->bus = bus;
  this->type = type;
  this->pressure_resolution = pressure_resolution;
  this->pressure_max = pressure_max;
  this->state = State::STATE0;

  // Produce bitmasks to mask out undefined bits of both sensors. The pressure sensor's resolution
  // depends on the purchased option (-6, -7, -8 for 16-, 17-, and 18-bit resolution respectively),
  // and the temperature sensor is always 16-bit resolution. Note that the *lower* bits are the unused
  // bits, so that it is the *UPPER* bits that should be kept and the lower ones discarded.
  pressure_resolution_mask    = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - pressure_resolution)) - 1);
  temperature_resolution_mask = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - TEMPERATURE_RESOLUTION)) - 1);

  // Store a few scaling factors depending on the pressure sensor output type.
  switch (type) {
    case GAGE:
      pressure_zero_ref = 0.1 * FULL_SCALE_REF;
      pressure_range = pressure_max;
      break;
    case DIFFERENTIAL:
      pressure_zero_ref = 0.5 * FULL_SCALE_REF;
      pressure_range = pressure_max * 2;
      break;
  }
}


AllSensors_DLHR::AllSensors_DLHR(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk, 
                                 SensorType type, SensorResolution pressure_resolution, float pressure_max) :
  pressure_unit(PressureUnit::IN_H2O),     
  temperature_unit(TemperatureUnit::CELCIUS)
{
        this->_cs = spi_cs;
        this->_mosi = spi_mosi;
        this->_miso = spi_miso;
        this->_sck = spi_clk;
    this->bus = NULL;
    this->type = type;
    this->pressure_resolution = pressure_resolution;
    this->pressure_max = pressure_max;
    this->state = State::STATE0;

    // Produce bitmasks to mask out undefined bits of both sensors. The pressure sensor's resolution
    // depends on the purchased option (-6, -7, -8 for 16-, 17-, and 18-bit resolution respectively),
    // and the temperature sensor is always 16-bit resolution. Note that the *lower* bits are the unused
    // bits, so that it is the *UPPER* bits that should be kept and the lower ones discarded.
    pressure_resolution_mask    = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - pressure_resolution)) - 1);
    temperature_resolution_mask = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - TEMPERATURE_RESOLUTION)) - 1);

    // Store a few scaling factors depending on the pressure sensor output type.
    switch (type) {
        case GAGE:
        pressure_zero_ref = 0.1 * FULL_SCALE_REF;
        pressure_range = pressure_max;
        break;
        case DIFFERENTIAL:
        pressure_zero_ref = 0.5 * FULL_SCALE_REF;
        pressure_range = pressure_max * 2;
        break;
    }
}


AllSensors_DLHR::AllSensors_DLHR(uint8_t spi_cs, SensorType type, SensorResolution pressure_resolution, float pressure_max):
  pressure_unit(PressureUnit::IN_H2O),     
  temperature_unit(TemperatureUnit::CELCIUS)
{
    this->_cs = spi_cs;
    this->bus = NULL;
    this->type = type;
    this->pressure_resolution = pressure_resolution;
    this->pressure_max = pressure_max;
    this->state = State::STATE0;

    // Produce bitmasks to mask out undefined bits of both sensors. The pressure sensor's resolution
    // depends on the purchased option (-6, -7, -8 for 16-, 17-, and 18-bit resolution respectively),
    // and the temperature sensor is always 16-bit resolution. Note that the *lower* bits are the unused
    // bits, so that it is the *UPPER* bits that should be kept and the lower ones discarded.
    pressure_resolution_mask    = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - pressure_resolution)) - 1);
    temperature_resolution_mask = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - TEMPERATURE_RESOLUTION)) - 1);

    // Store a few scaling factors depending on the pressure sensor output type.
    switch (type) {
        case GAGE:
        pressure_zero_ref = 0.1 * FULL_SCALE_REF;
        pressure_range = pressure_max;
        break;
        case DIFFERENTIAL:
        pressure_zero_ref = 0.5 * FULL_SCALE_REF;
        pressure_range = pressure_max * 2;
        break;
    }
}


bool AllSensors_DLHR::begin(void){
  if(bus == NULL ){ // SPI mode
    pinMode(this->_cs, OUTPUT);
    digitalWrite(this->_cs, HIGH);
    this->state = State::STATE0;
    //this->process_finished = 0;
        //Serial.println("no SPI bus, invoking SPI.begin();...");
        //SPI.setClockDivider(SPI_CLOCK_DIV16);
        //SPI.begin();
  }
  else{}
  return true;
}




void AllSensors_DLHR::startMeasurement(MeasurementType measurement_type) {
    if(bus == NULL ){ // SPI mode
        SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));
        digitalWrite (this->_cs, LOW);
        SPI.transfer((uint8_t) measurement_type);
        SPI.transfer(0x00);
        SPI.transfer(0x00);
        digitalWrite (this->_cs, HIGH);
        SPI.endTransaction();
    }
    else{ //i2c mode
        bus->beginTransmission(I2C_ADDRESS);
        bus->write((uint8_t) measurement_type);
        bus->write(0x00);
        bus->write(0x00);
        bus->endTransmission();
    }
}

uint8_t AllSensors_DLHR::readStatus() {
    if(bus ==NULL ){// SPI mode 
        SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));
        digitalWrite (this->_cs, LOW);
            status = SPI.transfer(0xF0);
            SPI.transfer(0x00);
            SPI.transfer(0x00);
        digitalWrite (this->_cs, HIGH);
        SPI.endTransaction();
    }
    else{//i2c mode
        bus->requestFrom(I2C_ADDRESS, READ_STATUS_LENGTH);
        status = bus->read();
        bus->endTransmission();
    }
  return status;
}

bool AllSensors_DLHR::readData(bool wait) {
try_again:
    if(bus ==NULL ){ //spi mode
            SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));
            digitalWrite (this->_cs, LOW);
            status = SPI.transfer(0xF0); 
            if (isError(status)) {
                // An ALU or memory error occurred.
                digitalWrite (this->_cs, HIGH);
                SPI.endTransaction();
                goto error;
            }
            if (isBusy(status)) {
                // The sensor is still busy; either retry or fail.
                digitalWrite (this->_cs, HIGH);
                SPI.endTransaction();
                if (wait) {
                // Wait just a bit so that we don't completely hammer the bus with retries.
                _delay_us(100);
                goto try_again;
                }
                goto error;
            }
            *((uint8_t *)(&raw_p)+2) = SPI.transfer(0x00);
            *((uint8_t *)(&raw_p)+1) = SPI.transfer(0x00);
            *((uint8_t *)(&raw_p)+0) = SPI.transfer(0x00);

            // Read the 24-bit (high 16 bits defined) of raw temperature data.
            *((uint8_t *)(&raw_t)+2) = SPI.transfer(0x00);
            *((uint8_t *)(&raw_t)+1) = SPI.transfer(0x00);
            *((uint8_t *)(&raw_t)+0) = SPI.transfer(0x00);

            digitalWrite (this->_cs, HIGH);
        SPI.endTransaction();
        pressure = convertPressure(transferPressure(raw_p & pressure_resolution_mask));
        temperature = convertTemperature(transferTemperature(raw_t & temperature_resolution_mask));
        return isError(status);
    }
    else{   //i2c mode
            bus->requestFrom(I2C_ADDRESS, (uint8_t) (READ_STATUS_LENGTH + READ_PRESSURE_LENGTH + READ_TEMPERATURE_LENGTH));
            // Read the 8-bit status data.
            status = bus->read();
            if (isError(status)) {
                // An ALU or memory error occurred.
                bus->endTransmission();
                goto error;
            }
            if (isBusy(status)) {
                // The sensor is still busy; either retry or fail.
                bus->endTransmission();
                if (wait) {
                // Wait just a bit so that we don't completely hammer the bus with retries.
                _delay_us(100);
                goto try_again;
                }
                goto error;
            }
            // Read the 24-bit (high 16-18 bits defined) of raw pressure data.
            *((uint8_t *)(&raw_p)+2) = bus->read();
            *((uint8_t *)(&raw_p)+1) = bus->read();
            *((uint8_t *)(&raw_p)+0) = bus->read();

            // Read the 24-bit (high 16 bits defined) of raw temperature data.
            *((uint8_t *)(&raw_t)+2) = bus->read();
            *((uint8_t *)(&raw_t)+1) = bus->read();
            *((uint8_t *)(&raw_t)+0) = bus->read();

            bus->endTransmission();

            pressure = convertPressure(transferPressure(raw_p & pressure_resolution_mask));
            temperature = convertTemperature(transferTemperature(raw_t & temperature_resolution_mask));
            return isError(status);
        }

error:
  pressure = NAN;
  temperature = NAN;

  return true;
}

bool AllSensors_DLHR::readDataAsynchro(MeasurementType measurement_type){
   bool dataReady = false;
   
   switch (this->state){
      case State::STATE0:
        startMeasurement(measurement_type);  //start conversion
        this->state = State::STATE1;
        break;

      case State::STATE1:
        if(bus==NULL){//spi mode
              SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));
              digitalWrite (this->_cs, LOW);
              status = SPI.transfer(0xF0); 
              if (isError(status)) {
                  // An ALU or memory error occurred.
                  digitalWrite (this->_cs, HIGH);
                  SPI.endTransaction();
                    this->pressure = NAN;
                    this->temperature = NAN;
                    this->state = State::STATE0;
                    dataReady = true;
                    break;
              }
              if (isBusy(status)) {
                // The sensor is still busy; either retry or fail.
                digitalWrite (this->_cs, HIGH);
                SPI.endTransaction();
              }
              else{
                  digitalWrite (this->_cs, HIGH);
                  SPI.endTransaction();
                this->state = State::STATE2;
              }
              break;
        }
        else{//i2c mode
              bus->requestFrom(I2C_ADDRESS, (uint8_t) (READ_STATUS_LENGTH));
              // Read the 8-bit status data.
              status = bus->read();
              if (isError(status)) {
                // An ALU or memory error occurred.
                bus->endTransmission();
                    this->pressure = NAN;
                    this->temperature = NAN;
                    this->state = State::STATE0;
                    dataReady = true;
                    break;
              }
              if (isBusy(status)) {
                // The sensor is still busy; either retry or fail.
                bus->endTransmission();
              }
              else{
                bus->endTransmission();
                this->state = State::STATE2;
              }
            break;
        }
        
      case State::STATE2:
        if(bus==NULL){//spi mode
              SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));
              digitalWrite (this->_cs, LOW);
              status = SPI.transfer(0xF0);
              *((uint8_t *)(&raw_p)+2) = SPI.transfer(0x00);
              *((uint8_t *)(&raw_p)+1) = SPI.transfer(0x00);
              *((uint8_t *)(&raw_p)+0) = SPI.transfer(0x00);

              // Read the 24-bit (high 16 bits defined) of raw temperature data.
              *((uint8_t *)(&raw_t)+2) = SPI.transfer(0x00);
              *((uint8_t *)(&raw_t)+1) = SPI.transfer(0x00);
              *((uint8_t *)(&raw_t)+0) = SPI.transfer(0x00);
              digitalWrite (this->_cs, HIGH);
              SPI.endTransaction() ;
          }
        else{//i2c mode
              bus->requestFrom(I2C_ADDRESS, (uint8_t) (READ_STATUS_LENGTH + READ_PRESSURE_LENGTH + READ_TEMPERATURE_LENGTH));
              status = bus->read();
              // Read the 24-bit (high 16-18 bits defined) of raw pressure data.
              *((uint8_t *)(&raw_p)+2) = bus->read();
              *((uint8_t *)(&raw_p)+1) = bus->read();
              *((uint8_t *)(&raw_p)+0) = bus->read();

              // Read the 24-bit (high 16 bits defined) of raw temperature data.
              *((uint8_t *)(&raw_t)+2) = bus->read();
              *((uint8_t *)(&raw_t)+1) = bus->read();
              *((uint8_t *)(&raw_t)+0) = bus->read();
              
              bus->endTransmission();
            }
          this->pressure = convertPressure(transferPressure(raw_p & pressure_resolution_mask));
          this->temperature = convertTemperature(transferTemperature(raw_t & temperature_resolution_mask));
          this->state = State::STATE0;
          dataReady = true;
        break;
   }
  return dataReady;
}
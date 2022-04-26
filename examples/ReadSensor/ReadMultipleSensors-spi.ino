#include <Arduino.h>
#include <SPI.h>
#include <AllSensors_DLHR.h>


/*
Multiple AllSensors DLHR using spi  - 26/04/2022 - Sylvain Boyer
_______________________________________________________________________________________________________________________________
This example is using a forked of the library All Sensors from Jeremy Cole: https://github.com/sylvanoMTL/AllSensors_DLHR
The library has been modified with an asynchronous method that uses a state machine to avoid code blocking.
For instance, if a sensor return a NaN, the other sensors are able to carry on their own pressure measurement.

in this example:
* the AllSensor DLHR is used using SPI bus. (note that not all DLHR have this capability, this is dependant of the sensor package)
* the Chip Select pins of the DLHR sensor are pulled up to 3.3V using a 3.3 kOhm resistor
* the Chip Select of the sensor is wired to pin 2,3,4 and 5
* two modes are proposed: 
        - single sensor: do the readings using the orginal functions from Jeremy Cole (modified for SPI)
        - multiple sensor: do the readings using asynchronous functions
_______________________________________________________________________________________________________________________________
*/

//Uncomment this line to have a single sensor, keep commented
//#define SINGLE_SENSOR


#ifdef SINGLE_SENSOR
  #define CS_PIN 2          
  AllSensors_DLHR_F05D_8 gagePressure(CS_PIN);
#else
  #define PRESSURE_0_CS_PIN 2
  #define PRESSURE_1_CS_PIN 3
  #define PRESSURE_2_CS_PIN 4
  #define PRESSURE_3_CS_PIN 5
  
  const byte pressure_arrayLen = 4;
  AllSensors_DLHR_F05D_8  gagepressure_array[pressure_arrayLen] ={  
                                                                  AllSensors_DLHR_F05D_8(PRESSURE_0_CS_PIN),
                                                                  AllSensors_DLHR_F05D_8(PRESSURE_1_CS_PIN),
                                                                  AllSensors_DLHR_F05D_8(PRESSURE_2_CS_PIN),
                                                                  AllSensors_DLHR_F05D_8(PRESSURE_3_CS_PIN)};


  bool condition[pressure_arrayLen]={0};
  float pressure[pressure_arrayLen];
  float temperature[pressure_arrayLen];
#endif




void setup() {
  #ifdef SINGLE_SENSOR
    pinMode(CS_PIN, OUTPUT); // set the SS pin as an output
    SPI.begin();         // initialize the SPI library
    Serial.begin(115200);
    gagePressure.begin();
    gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
  #else
    SPI.begin();         // initialize the SPI library
    for(byte i=0;i<pressure_arrayLen;i++){
      gagepressure_array[i].begin();
    }
    for(byte i=0;i<pressure_arrayLen;i++){
      gagepressure_array[i].setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
    }
    Serial.begin(115200);
  #endif
}

void loop(){
  #ifdef SINGLE_SENSOR
    gagePressure.startMeasurement();
    //Serial.println(gagePressure.readStatus());
    gagePressure.readData(true);
    Serial.print("Pressure: ");
    Serial.print(gagePressure.pressure);
    Serial.print(" Temperature: ");
    Serial.println(gagePressure.temperature);
    delay(20);
  #else
    // initiate the measurements and get results when available
    for(byte i=0;i<pressure_arrayLen;i++){
        condition[i]=gagepressure_array[i].readDataAsynchro();
        if(condition[i]) { 
          pressure[i] = gagepressure_array[i].pressure;
          temperature[i]=gagepressure_array[i].temperature;
          condition[i]=0;
        }
    }

    //print the results to the serial port
    for(byte i=0;i<pressure_arrayLen;i++){
        Serial.print(pressure[i]);Serial.print("\t");
    }
    for(byte i=0;i<pressure_arrayLen;i++){
        Serial.print(temperature[i]);Serial.print("\t");
    }
    Serial.println();
    delay(20);
  #endif
}


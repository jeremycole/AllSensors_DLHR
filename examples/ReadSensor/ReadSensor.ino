#include <Wire.h>

#include <AllSensors_DLHR.h>

AllSensors_DLHR_L10G_8 gagePressure(&Wire);

void setup() {
  Serial.begin(115200);

  Wire.begin();

  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
}

void loop() {
  gagePressure.startMeasurement();
  gagePressure.readData(true);
  Serial.print("Pressure: ");
  Serial.print(gagePressure.pressure);
  Serial.print(" Temperature: ");
  Serial.println(gagePressure.temperature);

  delay(100);
}

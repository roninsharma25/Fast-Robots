#include <Wire.h>
#include "SparkFun_VL53L1X.h"

// ToF Sensors
SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

int motorIndex;

void setupTOF() {
  digitalWrite(6, LOW);
  Wire.begin();
  distanceSensor2.setI2CAddress(0x32); // set a different I2C address for the second sensor
  digitalWrite(6, HIGH);

  resetDistanceArray();
  motorIndex = 0;

  //Serial.println("Sensors online!");

  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();
}


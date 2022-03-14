#include "setup.h"
#include "mainBLE.h"
#include "motors.h"

unsigned long startTime;
float acc;
ICM_20948_I2C myICM;

void setup() {
  Serial.begin(9600);

  initializeSensors();
  setupMotors();
  startBLE();

}

void loop() {

  // Send data over bluetooth
  int type = sendDataBLE(myICM);
  if (type == 10) { // Update BLE
    acc = 0;
  }
  delay(100);

}

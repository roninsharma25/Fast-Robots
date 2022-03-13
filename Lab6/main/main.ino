#include "mainBLE.h"
#include "mainSensors.h"
#include "motors.h"

unsigned long startTime;

float acc;

void setup() {
  Serial.begin(9600);

  initializeSensors();
  setupMotors();
  startBLE();

}

void loop() {

  // Send data over bluetooth
  sendDataBLE(myICM);
  delay(100);

}

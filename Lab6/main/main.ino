#include "mainBLE.h"
#include "setupSensors.h"
#include "motors.h"

unsigned long startTime;

float acc;

void setup() {
  Serial.begin(9600);

  initializeSensors();
  startBLE();

}

void loop() {

  // Send data over bluetooth
  sendDataBLE();
  delay(100);

}


#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

unsigned long startTime;
unsigned long rangingTime;

void setup(void)
{
  digitalWrite(4, LOW);
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  distanceSensor2.setI2CAddress(0x32); // set a different I2C address for the second sensor
  digitalWrite(4, HIGH);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  if (distanceSensor2.begin() != 0)
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
  distanceSensor.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();

  distanceSensor.setTimingBudgetInMs(20);
  distanceSensor2.setTimingBudgetInMs(20);
  
  Serial.println("Sensors online!");
}

void loop(void)
{
  startTime = millis();
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();
  
  while (!(distanceSensor.checkForDataReady() && distanceSensor2.checkForDataReady()))
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  rangingTime = millis() - startTime;

  Serial.print("Distance 1 (mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  //Serial.print("\tDistance(ft): ");
  //Serial.print(distanceFeet, 2);

  //Serial.println();

  Serial.print(" | Distance 2 (mm): ");
  Serial.print(distance2);

  float distanceInches2 = distance2 * 0.0393701;
  float distanceFeet2 = distanceInches2 / 12.0;

  //Serial.print("\tDistance 2 (ft): ");
  //Serial.print(distanceFeet2, 2);

  Serial.print(" | Ranging Time (msec): ");
  Serial.println(rangingTime);

}

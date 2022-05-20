#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// ToF Sensors
SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

int motorIndex;

// Store data for PID debugging
bool noPID;
int numberDistanceMeasurements = 2000;
int distances[2000];
int distanceIndex;
bool distanceMeasurementsDone;

float tof1;
float tof2;

void resetDistanceArray() {
  distanceIndex = 0;
  distanceMeasurementsDone = false;
}

void setupTOF() {
  digitalWrite(6, LOW);
  Wire.begin();
  distanceSensor2.setI2CAddress(0x32); // set a different I2C address for the second sensor
  digitalWrite(6, HIGH);

  resetDistanceArray();
  motorIndex = 0;

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

//  if (distanceSensor2.begin() != 0)
//  {
//    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
//    //while (1)
//      ;
//  }
  

  //distanceSensor.setDistanceModeShort();
  distanceSensor.setDistanceModeLong();

  //distanceSensor.setTimingBudgetInMs(20);
  //distanceSensor2.setTimingBudgetInMs(20);

  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();
}

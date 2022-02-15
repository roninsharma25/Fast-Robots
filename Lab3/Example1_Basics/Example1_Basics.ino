/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "Math.h"

//#define USE_SPI       // Uncomment this to use SPI

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

int LPF_size = 250;
int LPF_index;
double alpha_acc = 0.14; // update based on cutoff frequency
double alpha_gyro = 0.04; // same value as 

int count;
int max_count = 500;
unsigned long start_time;
float time_axis[500];
float pitch_values[500];

float prev_acc_x, prev_acc_y, prev_acc_z, acc_x, acc_y, acc_z;
double pitch, acc_pitch, roll, acc_roll, prev_pitch, prev_roll, gyro_pitch, gyro_roll, gyro_yaw, comp_pitch, comp_roll;
double dt = 30; // 30 sec delay in the loop


void setup()
{

  SERIAL_PORT.begin(9600);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    //SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      //SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  LPF_index = 1;
  count = 0;
  start_time = millis();
}

void loop()
{
//  start_dt = millis();
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    //dt = millis() - start_dt;
    delay(30);
  }
  else
  {
    //SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

  // Store the previous values
  prev_acc_x = acc_x;
  prev_acc_y = acc_y;
  prev_acc_z = acc_z;

  prev_pitch = pitch;
  prev_roll = roll;

  // Obtain new pitch and roll values from the accelerometer
  acc_x = myICM.accX();
  acc_y = myICM.accY();
  acc_z = myICM.accZ();

  // Current Pitch Values Calculated From the Accelerometer Data
  acc_pitch = atan2(acc_x, acc_z);
  acc_roll = atan2(acc_y, acc_z);

  // LPF pitch and roll
  pitch = (alpha_acc * acc_pitch) + (1 - alpha_acc) * prev_pitch;
  roll = (alpha_acc * acc_roll) + (1 - alpha_acc) * prev_roll;

  // Calculate pitch (y) and roll (x) from the gyroscope
  gyro_pitch -= myICM.gyrY() * dt;
  gyro_roll -= myICM.gyrX() * dt;
  gyro_yaw -= myICM.gyrZ() * dt;

  // Complementary Filter - Gyroscope
  comp_pitch = ( (comp_pitch + myICM.gyrY() * dt) * (1 - alpha_gyro) ) + (acc_pitch * alpha_gyro);
  comp_roll = ( (comp_roll + myICM.gyrX() * dt) * (1 - alpha_gyro) ) + (acc_roll * alpha_gyro);


  Serial.print("Acc_Pitch:");
  Serial.print(acc_pitch);
  Serial.print(" Filtered_Pitch:");
  Serial.print(pitch);
//  Serial.print(" Gyro_Pitch:");
//  Serial.print(gyro_pitch);
  Serial.print(" Comp_Pitch:");
  Serial.print(comp_pitch);
  Serial.print(" Acc_Roll:");
  Serial.print(acc_roll);
  Serial.print(" Filtered_Roll:");
  Serial.print(roll);
//  Serial.print(" Gyro_Roll:");
//  Serial.println(gyro_roll);
  Serial.print(" Comp_Roll:");
  Serial.println(comp_roll);

//  Serial.print("Gyro Pitch: ");
//  Serial.print(gyro_pitch);
//  Serial.print(" | Gyro Roll: ");
//  Serial.print(gyro_roll);
//  Serial.print(" | Gyro Yaw: ");
//  Serial.println(gyro_yaw);


  // Convert Magnetometer Data into a Yaw Angle
  double xm = myICM.magX()*cos(pitch) - myICM.magY()*sin(roll)*sin(pitch) + myICM.magZ()*cos(roll)*sin(pitch);
  double ym = myICM.magY()*cos(roll) + myICM.magZ()*sin(roll);
  double yaw = atan2(ym, xm);

  //delay(100);


  
//  pitch_values[count] = (float) pitch;
//  time_axis[count] = (float) (millis() - start_time);
//  count += 1;
//
//  if(count >= max_count) {
//    for (int i = 0; i < max_count; i++) {
//      Serial.print(pitch_values[i]);
//      Serial.print(", ");
//    }
//
//    Serial.println();
//    
//    /*for (int i = 0; i < max_count; i++) {
//      Serial.print(time_axis[i]);
//      Serial.print(", ");
//    }*/
//    while(1) {};
//  }

}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

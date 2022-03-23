#include "RobotCommand.h"
#include "ICM_20948.h"
#include "Math.h"
#include "motors.h"
#include "ble.h"
#include "sensors.h"
#include "kf.h"

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


// RX
RobotCommand robot_cmd(":|");

// TX
float tx_float_value = 0.0;

// PID Constants
float setpoint, k_p, k_i, k_d;
float cumulativeError, prevError;

unsigned long startTime;
unsigned long rangingTime;

// Motors
float motorSpeed;
float motorSpeed1;
float motorSpeed2;

float motorValues[2000];

BLEDevice central;


enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    MOVE_FORWARD,
    STOP_ROBOT,
    GET_IMU,
    UPDATE_PID
};

void getIMUCase() {
//  myICM.getAGMT();
//  float accX = myICM.accX()/1000;
//
//  char char_arr[MAX_MSG_SIZE];
//
//  writeTXFloat4(accX);

  float res = getTOF2();
  writeTXFloat4(res);
}

void sendDistanceData() {
  /*numberDistanceMeasurements = 500;
int distances[numberDistanceMeasurements];
int distanceIndex;
bool distanceMeasurementsDone;*/

  for (int i = 0; i < numberDistanceMeasurements; i++) {
    writeTXFloat3(distances[i]); // TOF2
    delay(100);
  }
  
}

void
handle_command()
{   
    // Set the command string from the characteristic value
    // UPDATE -- PASS TO BLE HEADER FILE
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            // Use PING to toggle between writing motor PWM values
            if (startWritingPWM) {
              startWritingPWM = false;
            } else {
              startWritingPWM = true;
            }

            int startingDistance;
            //success = robot_cmd.get_next_value(startingDistance);
            x(0,0) = 800; //startingDistance;
            x(0,1) = 0;

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            float float_a, float_b, float_c;

            // Extract the first value from the command string as a float
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the second value from the command string as a float
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;

            // Extract the third value from the command string as a float
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            // Display the three floats
            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);
            
            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        /*
         * MOVE_FORWARD
         */
        case MOVE_FORWARD:

            Serial.println("IN CASE MOVE FORWARD");

            // reset the distance array
            resetDistanceArray();

            int motor_speed_1, motor_speed_2, forward, doPID;

            // Extract the next value from the command string as an int
            success = robot_cmd.get_next_value(motor_speed_1);
            if (!success)
                return;

            // Extract the next value from the command string as an int
            success = robot_cmd.get_next_value(motor_speed_2);
            if (!success)
                return;

            // Extract the next value from the command string as an int
            success = robot_cmd.get_next_value(forward); // forward or backwards
            if (!success)
                return;

            // Extract the next value from the command string as an int
            success = robot_cmd.get_next_value(doPID); // determine whether to do PID or not
            if (!success)
                return;

            noPID = doPID == 1;

            Serial.println(motor_speed_1);
            Serial.println(motor_speed_2);
            Serial.println(forward);

            motorSpeed = motor_speed_1;
            motorSpeed1 = motor_speed_1;
            motorSpeed2 = motor_speed_2;

            moveForwardCase(motor_speed_1, motor_speed_2, forward);
            startTime = millis();

            break;

        /*
         * STOP_ROBOT
         */
        case STOP_ROBOT:

          motorSpeed = 0;
          startedMoving = false;
          stopRobotFast();
          break;

        /*
         * GET_IMU
         */
        case GET_IMU: // ACTUALLY RETURNS FRONT TOF SENSOR VALUE
          //int res = getTOF2();
          //writeTXFloat4(res);
          getIMUCase();
          break;
        
         /*
         * UPDATE_PID
         */
        case UPDATE_PID:
            float float_aa, float_bb, float_cc, float_dd, float_uu, float_zz;

            // Extract the first value from the command string as a float
            success = robot_cmd.get_next_value(float_aa);
            if (!success)
                return;

            // Extract the second value from the command string as a float
            success = robot_cmd.get_next_value(float_bb);
            if (!success)
                return;

            // Extract the third value from the command string as a float
            success = robot_cmd.get_next_value(float_cc);
            if (!success)
                return;

            // Extract the fourth value from the command string as a float
            success = robot_cmd.get_next_value(float_dd);
            if (!success)
                return;

            // Extract the fifth value from the command string as a float
            success = robot_cmd.get_next_value(float_uu);
            if (!success)
                return;

            // Extract the sixth value from the command string as a float
            success = robot_cmd.get_next_value(float_zz);
            if (!success)
                return;

            noPID = false;
            startedMoving = false;

            setpoint = float_aa;
            k_p = float_bb;
            k_i = float_cc;
            k_d = float_dd;

            // Set KF parameters
            sig_u(0,0) = float_uu;
            sig_u(1,1) = float_uu;
            
            sig_z(0,0) = float_zz;

            break;

        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

// Motor Starting Values
int m1_val;
int m2_val;

float acc;

void setup() {
  Serial.begin(9600);

  setupTOF();

  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif

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

  setupBLE();
  setupMotors();

  noPID = true;
  cumulativeError = 0;
  prevError = 0;
  
  // Set starting motor values
  m1_val = 50;
  m2_val = 50;
}

void loop() {

  // Check sensors
//  myICM.getAGMT();
//  delay_(100, central);
//  float accX = myICM.accX()/1000;
//  delay_(100, central);

  // Send data over bluetooth
  
  central = BLE.central();

  tof2 = getTOF2();

  if (central) {
    
    if (checkRXCharString()) {
        handle_command();
    }

    // still send data if the robot is moving forward without performing PID
    if (startWritingPWM) {
      writeTXFloatMotor(motorSpeed); // write PWM value to the corresponding float characteristic
      tof2 = getTOF2();
      if (tof2 != -1000) writeTXFloat3(tof2);
    }

    if (kfPWMReady) {
      // Write KF PWM value to the corresponding float characteristic
      writeTXFloatKFMOTORPWM(x(1,0) / 100);
      kfPWMReady = false;
    }
  }

  if (noPID == false && startedMoving) {
    
    Serial.println("Doing PID");

    //if (movingForward) {
    tof2 = getTOF2();
    float kfOut = performKF(tof2, motorSpeed);
    writeTXFloatKFTOF(kfOut);
    
    PID(tof2, millis() - startTime, kfOut);
    //}
    
    //distances[distanceIndex] = tof2;
    //distanceIndex++;

    //if (distanceIndex >= numberDistanceMeasurements) {
    //  distanceMeasurementsDone = true;
    //  startedMoving = false;
      //sendDistanceData();
    //}
  } else {
    writeTXFloatKFTOF(x(0,0));
    Serial.println(x(0,0));
  }
}

void PID(float sensorValue, unsigned long dt, float kfOut) {

  if (kfOut <= setpoint + 50 && kfOut >= setpoint - 10) { // stop the robot
    stopRobotFast();
    startTime = millis();
  } else {

      float error = kfOut - setpoint;

      // dir is 0 when going forward and 1 when going backwards
      // error < 0 --> passed setpoint so go backwards
      int dir = 0;
      if (error < 0) {
        dir = 1;
      }

      // P
      float delta_p = k_p * error;

      // I
      cumulativeError += error * dt;
      float delta_i = k_i * cumulativeError;

      // D
      float errorChange = error - prevError;
      float delta_d = k_d * errorChange / dt;

      motorSpeed = abs(delta_p + delta_i + delta_d);

      // Deadband and max PWM signal thresholding
      if (motorSpeed < 50) {
        motorSpeed = 50;
      } else if (motorSpeed > 100) {
        motorSpeed = 100;
      }

      // Write motor speed to the corresponding float characteristic
      writeTXFloatMotor(motorSpeed);

      startTime = millis();
      prevError = error;

      // write new speeds
      moveForwardCase(motorSpeed, motorSpeed, dir);

  }
}

int getTOF1() {
  distanceSensor.startRanging();
  int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor (in mm)
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  return distance;
}

int getTOF2() { // Front sensor

  int distance2;
  if (distanceSensor2.checkForDataReady()) {
    distance2 = distanceSensor2.getDistance(); // Get the result of the measurement from the sensor (in mm)
    distanceSensor2.clearInterrupt();

    Serial.println(distance2);
  } else {
    distance2 = -1000;
  }
  
  return distance2;
}

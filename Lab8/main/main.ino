#include "RobotCommand.h"
#include "ICM_20948.h"
#include "Math.h"
#include "motors.h"
#include "ble.h"
#include "sensors.h"
#include "kf.h"

// RX
RobotCommand robot_cmd(":|");

// TX
float tx_float_value = 0.0;

// PID Constants
float setpoint, k_p, k_i, k_d;
float cumulativeError, prevError;

unsigned long startTime;
unsigned long rangingTime;

// Stunt
unsigned long stuntStartTime;
unsigned long flipFinishedTime;
bool startedStunt = false;
bool flipFinished = false;
bool allDone = false;


void getIMUCase() {
  myICM.getAGMT();
  float accX = myICM.accX()/1000;

  char char_arr[MAX_MSG_SIZE];

  writeTXFloat4(accX);
}

void sendDistanceData() {
  for (int i = 0; i < numberDistanceMeasurements; i++) {
    writeTXFloat3(distances[i]); // TOF2
    delay(100);
  }
}

void
handle_command()
{   
    // Set the command string from the characteristic value
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
            int startingDistance, clearAllDone, performFlip;
            success = robot_cmd.get_next_value(startingDistance);
            success = robot_cmd.get_next_value(clearAllDone);
            success = robot_cmd.get_next_value(performFlip);

            // Use PING to toggle between writing motor PWM values
            if (startWritingPWM) {
              startWritingPWM = false;
            } else {
              startWritingPWM = true;
            }

            x(0,0) = startingDistance;
            x(0,1) = 0;

            if (clearAllDone == 1) { // reset
              allDone = false;
            }

            if (performFlip == 1) { // start the flip
              stopRobotFast();
              startedStunt = true;
              stuntStartTime = millis();
            } else {
              startedStunt = false;
              flipFinished = false;
            }

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
  
  central = BLE.central();

  //tof2 = getTOF2();

  if (central) {
    
    if (checkRXCharString()) {
        handle_command();
    }
  }

  if (central && !allDone) {

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

  if (!allDone && !startedStunt && !noPID && startedMoving) {

    Serial.println("Haven't detected wall");

    tof2 = getTOF2();
    float kfOut = performKF(tof2, motorSpeed);
    writeTXFloatKFTOF(kfOut);
    
    PID(tof2, millis() - startTime, kfOut);

  } else if (!allDone && startedStunt) {

    Serial.println("Performing stunt");
    
    writeTXFloatKFTOF(x(0,0));

    if (!flipFinished) {
        if ( (millis() - stuntStartTime) > 2000 ) { // allocate 2 seconds for the flip
            flipFinished = true;
            flipFinishedTime = millis();
        }
    } else { // flip is finished
      
      // Move backwards for two seconds
      if ( (millis() - flipFinishedTime) > 2000 ) {
        
        allDone = true;
        stopRobot();

      } else {

        moveForwardCase(200, 200, 0); // FLIP THIS DIRECTION AFTER INITIAL TESTS

      }
    }
  }
}

void PID(float sensorValue, unsigned long dt, float kfOut) {

  Serial.println("PIDing");

  if (kfOut <= setpoint + 50) { // start performing flip
    stopRobotFast();

    startedStunt = true;
    stuntStartTime = millis();
    
  } else if (kfOut <= setpoint + 50 && kfOut >= setpoint - 10) { // stop the robot
 
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
      } else if (motorSpeed > 255) {
        motorSpeed = 255;
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
  } else {
    distance2 = -1000;
  }

  Serial.println(distance2);
  //distanceSensor2.startRanging();
  
  return distance2;
}

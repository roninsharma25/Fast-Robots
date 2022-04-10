#include "RobotCommand.h"
#include "ICM_20948.h"
#include "Math.h"
#include "motors.h"
#include "ble.h"
#include "sensors.h"
#include "kf.h"

// RX
RobotCommand robot_cmd(":|");

// PID Constants
float setpoint, k_p, k_i, k_d;
float cumulativeError, prevError;

unsigned long startTime;
unsigned long previousTime;
unsigned long currentTime;
unsigned long rangingTime;

float prevGyroVal, currGyroVal, deltaGyro;

bool started = false;

void getIMUCase() {
  myICM.getAGMT();
  float accX = myICM.accX()/1000;

  char char_arr[MAX_MSG_SIZE];

  writeTXFloat4(accX);
}

void updateGyro() {
  myICM.getAGMT();
  currentTime = millis();

  // Yaw angle
  currGyroVal -= myICM.gyrZ() * (currentTime - previousTime) / 1000;
  Serial.println(currGyroVal);

  writeTXFloat4(currGyroVal);

  // Update time
  previousTime = currentTime;
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
            int startingDistance, toggleUpdate, clearAllDone, performFlip;
            success = robot_cmd.get_next_value(startingDistance);
            success = robot_cmd.get_next_value(toggleUpdate);
            success = robot_cmd.get_next_value(clearAllDone);
            success = robot_cmd.get_next_value(performFlip);

            // Use PING to toggle between writing motor PWM values
            if (startWritingPWM) {
              startWritingPWM = false;
            } else {
              startWritingPWM = true;
            }

            Serial.println(toggleUpdate);
            started = toggleUpdate ? !started : started;

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
            previousTime = millis();

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
            int doPID2;

            // Extract the first value from the command string as a float
            success = robot_cmd.get_next_value(float_aa);

            // Extract the second value from the command string as a float
            success = robot_cmd.get_next_value(float_bb);

            // Extract the third value from the command string as a float
            success = robot_cmd.get_next_value(float_cc);

            // Extract the fourth value from the command string as a float
            success = robot_cmd.get_next_value(float_dd);

            // Extract the fifth value from the command string as a float
            success = robot_cmd.get_next_value(float_uu);

            // Extract the sixth value from the command string as a float
            success = robot_cmd.get_next_value(float_zz);

            success = robot_cmd.get_next_value(doPID2);

            noPID = doPID2 == 0; // doPID = 1 when the robot should move
            startedMoving = !noPID2;

            setpoint = float_aa;
            k_p = float_bb;
            k_i = float_cc;
            k_d = float_dd;

            previousTime = millis();
            updateGyro();
            prevGyroVal = currGyroVal;

            break;

        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
          case TURN:
            float forwardSpeed, backwardSpeed, dir;

            // Extract speed and direction
            success = robot_cmd.get_next_value(forwardSpeed);
            success = robot_cmd.get_next_value(backwardSpeed);
            success = robot_cmd.get_next_value(dir);

            turn(forwardSpeed, backwardSpeed, dir);


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

  if (central) {

    if (checkRXCharString()) {
        handle_command();
    }

    
    if (!noPID && startedMoving) {
      PID(millis() - startTime);
    }

    if (started) {
      tof2 = getTOF2();
      writeTXFloat3(tof2);
      writeTXFloatMotor(motorSpeed); // write PWM value to the corresponding float characteristic
      updateGyro();

    }
  }
}

void PID(unsigned long dt) {
    
  if (currGyroVal >= setpoint - 5) { // stop the robot when it's within 5 degrees of finishing the turn
 
    stopRobotFast();
    writeTXFloat2(getTOF2());
    prevGyroVal = currGyroVal;
    turn(motorSpeed, 85, 1);

  } else {

      float error = abs(currGyroVal - prevGyroVal) - setpoint;

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
      if (motorSpeed < 150) {
        motorSpeed = 150;
      } else if (motorSpeed > 255) {
        motorSpeed = 255;
      }

      // Write motor speed to the corresponding float characteristic
      writeTXFloatMotor(motorSpeed);

      startTime = millis();
      prevError = error;

      // write new speeds
      turn(motorSpeed, 85, 1);
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
  distanceSensor2.startRanging();
  int distance2 = distanceSensor2.getDistance();
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  
  return distance2;
}

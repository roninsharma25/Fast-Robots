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

// Variables for stopping and restarting movement
bool stoppedRobot = false;
unsigned long stoppedRobotTime;

float prevGyroVal, currGyroVal, deltaGyro, timeBuffer;
float backwardSpeed;

bool started = false;
bool doPID = false;

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
  //Serial.println(currGyroVal);

  //writeTXFloat4(currGyroVal);

  // Update time
  previousTime = currentTime;

  //Serial.println("IN UPDATE GYRO");
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
            // Use PING to toggle between writing motor PWM and sensor values
            startWritingPWM = !startWritingPWM;
            started = !started;

            break;

        /*
         * MOVE_FORWARD
         */
        case MOVE_FORWARD:

            // reset the distance array
            resetDistanceArray();

            int motor_speed_1, motor_speed_2, forward, PID;

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
            success = robot_cmd.get_next_value(PID); // determine whether to do PID or not
            if (!success)
                return;

            doPID = PID == 1;

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

          doPID = false;
          motorSpeed = 0;
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
            float float_aa, float_bb, float_cc, float_dd, PID2, time_;
            success = robot_cmd.get_next_value(float_aa);
            success = robot_cmd.get_next_value(float_bb);
            success = robot_cmd.get_next_value(float_cc);
            success = robot_cmd.get_next_value(float_dd);
            success = robot_cmd.get_next_value(PID2);
            success = robot_cmd.get_next_value(time_);
            success = robot_cmd.get_next_value(backwardSpeed); // update the backward speed

            doPID = PID2 == 1; // doPID is true when PID should be performed
            timeBuffer = time_;
            
            setpoint = float_aa;
            k_p = float_bb;
            k_i = float_cc;
            k_d = float_dd;

            previousTime = millis();
            updateGyro();
            prevGyroVal = currGyroVal;
            stoppedRobot = false;
            

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

  // Set starting time buffer
  timeBuffer = 2000;
}

void loop() {
  
  central = BLE.central();

  if (central) {

    if (checkRXCharString()) {
        handle_command();
    }

    if (doPID) {
      PID(millis() - startTime);
    }

    if (started) {
      tof2 = getTOF2();
      writeTXFloat3(tof2); // write to TOF 2
      writeTXFloatMotor(motorSpeed); // write PWM value to the corresponding float characteristic
      updateGyro();
    }
  }
}

void PID(unsigned long dt) {

  float delta = abs(currGyroVal - prevGyroVal);

  if (stoppedRobot && (millis() - stoppedRobotTime > timeBuffer)) {  // turning time buffer has elapsed --> continue turning
    
    stoppedRobot = false;
    motorSpeed = 150;

  } else if (stoppedRobot) { // turning time buffer hasn't elapsed yet

    stopRobotFast();

  } else if (delta >= setpoint - 5) { // stop the robot when it's within 5 degrees of finishing the turn

    stopRobotFast();
    writeTXFloat2(getTOF2()); // write to TOF 1
    writeTXFloat4(currGyroVal); // write Gyroscope value
    prevGyroVal = currGyroVal;
    turn(motorSpeed, backwardSpeed, 1);

    stoppedRobot = true;
    stoppedRobotTime = millis();

  } else {

      float error = delta - setpoint;

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
      turn(motorSpeed, backwardSpeed, 1);
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

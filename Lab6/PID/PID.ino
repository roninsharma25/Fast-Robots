#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "ICM_20948.h"
#include "Math.h"

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


//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
#define BLE_UUID_TX_TOF1 "8fdf6466-8bd1-48e7-8744-814e57775ebd"
#define BLE_UUID_TX_TOF2 "d1ae58eb-8f6b-46b7-83f8-bbe541e772cc"
#define BLE_UUID_TX_IMU "73d4b8ab-890d-4e4c-b926-a6e294d50c9b"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);


BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);

BLEFloatCharacteristic tx_characteristic_float2(BLE_UUID_TX_TOF1, BLERead | BLENotify);
BLEFloatCharacteristic tx_characteristic_float3(BLE_UUID_TX_TOF2, BLERead | BLENotify);
BLEFloatCharacteristic tx_characteristic_float4(BLE_UUID_TX_IMU, BLERead | BLENotify);

BLEDevice central;

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

// PID Constants
float setpoint, k_p, k_i, k_d;

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
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

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

            int motor_speed_1, motor_speed_2, forward;

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

            Serial.println(motor_speed_1);
            Serial.println(motor_speed_2);
            Serial.println(forward);

            moveForwardCase(motor_speed_1, motor_speed_2, forward);

            break;

        /*
         * STOP_ROBOT
         */
        case STOP_ROBOT:
          
          stopRobot();
          break;

        /*
         * GET_IMU
         */
        case GET_IMU:

          getIMUCase();
          
          break;
        
         /*
         * UPDATE_PID
         */
        case UPDATE_PID:
            float float_a, float_b, float_c, float_d;

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

            // Extract the fourth value from the command string as a float
            success = robot_cmd.get_next_value(float_d);
            if (!success)
                return;

            setpoint = float_a;
            k_p = float_b;
            k_i = float_c;
            k_d = float_d;

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

// Motor 1
int m1_pin1 = 2;
int m1_pin2 = 3;

// Motor 2
int m2_pin1 = 14;
int m2_pin2 = 16;

// Motor Starting Values
int m1_val;
int m2_val;

unsigned long startTime;

float acc;

void setup() {
  Serial.begin(9600);

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


  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_float2);
  testService.addCharacteristic(tx_characteristic_float3);
  testService.addCharacteristic(tx_characteristic_float4);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  tx_characteristic_float.writeValue(0.0);

  tx_estring_value.clear();

  // Append the string literal "[->"
  tx_estring_value.append("[->");

  // Append the float value
  tx_estring_value.append(9.0);

  // Append the string literal "<-]"
  tx_estring_value.append("<-]");

  // Write the value to the characteristic
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  
  pinMode(m1_pin1, OUTPUT);
  pinMode(m1_pin2, OUTPUT);

  pinMode(m2_pin1, OUTPUT);
  pinMode(m2_pin2, OUTPUT);

  // Set starting motor values
  m1_val = 50;
  m2_val = 50;

  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

void delay_(int time_, BLEDevice ble) {
  unsigned long startTime = millis();
  while (millis() - startTime < time_) {
     int check = ble.connected();
  }
}

void loop() {

  // Check sensors
  myICM.getAGMT();
  delay_(100, central);
  float accX = myICM.accX()/1000;
  delay_(100, central);

  // Send data over bluetooth
  
  central = BLE.central();

  if (central) {
    tx_characteristic_float4.writeValue(accX);
    
    if (rx_characteristic_string.written()) {
        handle_command();
    }
  }
}


void moveForward(int speed1, int speed2) {
  analogWrite(m1_pin1, speed1);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, speed2);
}

void stopRobot() {
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);
}

void moveForwardCase(int speed1, int speed2, int forward) {
  
  if (forward == 0) {
    Serial.print("forward");

    // update speed1 and speed2 based on sensors
    
    analogWrite(m1_pin1, speed1);
    analogWrite(m1_pin2, 0);
    
    analogWrite(m2_pin1, 0);
    analogWrite(m2_pin2, speed2);
    
  } else {
    
    Serial.print("backward");
    analogWrite(m1_pin1, 0);
    analogWrite(m1_pin2, speed1);
    
    analogWrite(m2_pin1, speed2);
    analogWrite(m2_pin2, 0);
  }
}

void getIMUCase() {
  myICM.getAGMT();
  float accX = myICM.accX()/1000;

  char char_arr[MAX_MSG_SIZE];

  tx_characteristic_float4.writeValue(accX);

  Serial.print("Sent back: ");
  Serial.println(accX);
}

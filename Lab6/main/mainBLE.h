#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "utilsBLE.h"
#include "utilsPID.h"

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

// PID
struct PID p;
struct PIDInfo p_info;

//////////// Global Variables ////////////

void startBLE() {
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

    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

}

void getIMUCase(ICM_20948_I2C icm) {
  icm.getAGMT();
  float accX = icm.accX()/1000;

  char char_arr[MAX_MSG_SIZE];

  tx_characteristic_float4.writeValue(accX);

  Serial.print("Sent back: ");
  Serial.println(accX);
}

void
handle_command(ICM_20948_I2C icm)
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

          getIMUCase(icm);
          
          break;
        
         /*
         * UPDATE_PID
         */
        case UPDATE_PID:
            float float_aa, float_bb, float_cc, float_dd;

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
            
            p = setPID(float_aa, float_bb, float_cc, float_dd, 0);
            p_info = setPIDInfo(&m, 50, 50);
 
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

void sendDataBLE(ICM_20948_I2C icm) {
  central = BLE.central();

  float sendValue = 10;

  if (central) {
    tx_characteristic_float4.writeValue(sendValue);
    
    if (rx_characteristic_string.written()) {
        handle_command(icm);
    }
  }
}

#include "BLECStringCharacteristic.h"
#include <ArduinoBLE.h>

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

void setupBLE() {
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
}

bool checkRXCharString() {
    return rx_characteristic_string.written();
}

void writeTXFloat2(float val) {
    tx_characteristic_float2.writeValue(val);
}

void writeTXFloat3(float val) {
    tx_characteristic_float3.writeValue(val);
}

void writeTXFloat4(float val) {
    tx_characteristic_float4.writeValue(val);
}

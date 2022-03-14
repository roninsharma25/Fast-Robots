#include "ICM_20948.h"
#include "Math.h"

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

extern ICM_20948_I2C myICM;

void initializeSensors() {
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
}

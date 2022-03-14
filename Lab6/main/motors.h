#include "utilsMotor.h"

struct motorSpeeds m;

// Motor 1
int m1_pin1 = 2;
int m1_pin2 = 3;

// Motor 2
int m2_pin1 = 14;
int m2_pin2 = 16;

// Motor Starting Values
int m1_val = 50;
int m2_val = 50;

void setupMotors() {
    pinMode(m1_pin1, OUTPUT);
    pinMode(m1_pin2, OUTPUT);

    pinMode(m2_pin1, OUTPUT);
    pinMode(m2_pin2, OUTPUT);

    // Set starting motor values
    m = setMotorSpeeds(m1_val, 0, 0, m2_val);
}

void writePWM() {
    analogWrite(m1_pin1, m.speed1a);
    analogWrite(m1_pin1, m.speed1b);

    analogWrite(m2_pin1, m.speed2a);
    analogWrite(m2_pin2, m.speed2b);
}

void moveForward(int speed1, int speed2) {
    m = setMotorSpeeds(speed1, 0, 0, speed2);
    writePWM();
}

void moveBackward(int speed1, int speed2) {
    m = setMotorSpeeds(0, speed1, speed2, 0);
    writePWM();
}

void stopRobot() {
    m = setMotorSpeeds(0, 0, 0, 0);
    writePWM();
}

void moveForwardCase(int speed1, int speed2, int forward) {
  
  if (forward == 0) {

    Serial.print("forward");
    moveForward(speed1, speed2);
    
  } else {
    
    Serial.print("backward");
    moveBackward(speed1, speed2);

  }
}


// Motors
float motorSpeed;
float motorSpeed1;
float motorSpeed2;

float motorValues[2000];

bool movingForward;
bool startedMoving;
bool startWritingPWM;

// Motor 1
int m1_pin1 = 14; //2;
int m1_pin2 = 16; //3;

// Motor 2
int m2_pin1 = 2; //14;
int m2_pin2 = 3; //16;

void setupMotors() {
    movingForward = false;
    startedMoving = false;
    startWritingPWM = false;

    pinMode(m1_pin1, OUTPUT);
    pinMode(m1_pin2, OUTPUT);

    pinMode(m2_pin1, OUTPUT);
    pinMode(m2_pin2, OUTPUT);
}


void moveForwardCase(float speed1, float speed2, int forward) {

  movingForward = true;
  startedMoving = true;
  
  if (forward == 0) {

    analogWrite(m1_pin1, 0);
    analogWrite(m1_pin2, speed1);
    
    analogWrite(m2_pin1, speed2);
    analogWrite(m2_pin2, 0);
    
  } else {

    analogWrite(m1_pin1, speed1);
    analogWrite(m1_pin2, 0);
    
    analogWrite(m2_pin1, 0);
    analogWrite(m2_pin2, speed2);

  }
}

void stopRobotFast() {
  movingForward = false;
  analogWrite(m1_pin1, 255);
  analogWrite(m1_pin2, 255);

  analogWrite(m2_pin1, 255);
  analogWrite(m2_pin2, 255);
}

void stopRobot() {
  movingForward = false;
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);
}

void moveForward(int speed1, int speed2) {
  analogWrite(m1_pin1, speed1);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, speed2);
}

void turn(int forwardSpeed, int backwardSpeed, int dir) {
  if (dir) {
    analogWrite(m1_pin1, forwardSpeed);
    analogWrite(m1_pin2, 0);

    analogWrite(m2_pin1, backwardSpeed);
    analogWrite(m2_pin2, 0);
  } else {
    analogWrite(m1_pin1, 0);
    analogWrite(m1_pin2, backwardSpeed);

    analogWrite(m2_pin1, 0);
    analogWrite(m2_pin2, forwardSpeed);
  }
}

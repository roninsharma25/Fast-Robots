// Motor 1
int m1_pin1 = A2;
int m1_pin2 = A3;

// Motor 2
int m2_pin1 = A5;
int m2_pin2 = 6;

// Constants
int m1_val = 100;
int m2_val = 100;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  pinMode(m1_pin1, OUTPUT);
  pinMode(m1_pin2, OUTPUT);

  pinMode(m2_pin1, OUTPUT);
  pinMode(m2_pin2, OUTPUT);

}

void loop() {
  moveForward();
  delay(1000);
  stopRobot();
  delay(1000);
  turnRight();
  delay(1000);
  moveForward();
  delay(1000);
  turnLeft();
  delay(1000);
  moveForward();
  delay(1000);
}

void turnRight() {
  analogWrite(m1_pin1, m1_val);
  analogWrite(m1_pin2, m1_val);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);
}

void turnLeft() {
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, m2_val);
  analogWrite(m2_pin2, m2_val);
}

void moveForward() {
  analogWrite(m1_pin1, m1_val);
  analogWrite(m1_pin2, m1_val);

  analogWrite(m2_pin1, m2_val);
  analogWrite(m2_pin2, m2_val);
}

void stopRobot() {
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);
}

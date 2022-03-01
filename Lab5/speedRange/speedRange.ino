// Motor 1
int m1_pin1 = A2;
int m1_pin2 = A3;

// Motor 2
int m2_pin1 = A5;
int m2_pin2 = 6;

// Motor Starting Values
int m1_val;
int m2_val;

unsigned long startTime;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  pinMode(m1_pin1, OUTPUT);
  pinMode(m1_pin2, OUTPUT);

  pinMode(m2_pin1, OUTPUT);
  pinMode(m2_pin2, OUTPUT);

  // SSet starting motor values
  m1_val = 50;
  m2_val = 50;

}

void loop() {
  for (int i = 0; i < 10; i++) {
    startTime = millis(); // store the starting time
    
    moveForward(m1_val, m2_val);
    delay(100);

    // Use accelerometer to calculate the speed
    float acc = 5; // UPDATE
    acc = acc * (millis() - startTime);

    // Send data via Bluetooth


    // Ramp up speeds
    m1_val += 10;
    m2_val += 10;
    

    
  }
  
}


void moveForward(int speed1, int speed2) {
  analogWrite(m1_pin1, speed1);
  analogWrite(m1_pin2, speed1);

  analogWrite(m2_pin1, speed2);
  analogWrite(m2_pin2, speed2);
}

void stopRobot() {
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);
}

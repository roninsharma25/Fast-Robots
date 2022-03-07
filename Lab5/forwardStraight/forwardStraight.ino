// Motor 1
int m1_pin1 = 2;
int m1_pin2 = 3;

// Motor 2
int m2_pin1 = 14;
int m2_pin2 = 16;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  pinMode(m1_pin1, OUTPUT);
  pinMode(m1_pin2, OUTPUT);

  pinMode(m2_pin1, OUTPUT);
  pinMode(m2_pin2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);

  // Forward
  analogWrite(m1_pin1, 115);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 110);

  unsigned long startTime = millis();
  int count = 0;
  while (millis() - startTime < 1500) {
    count += 1;
  }

  // Stop
  analogWrite(m1_pin1, 0);
  analogWrite(m1_pin2, 0);

  analogWrite(m2_pin1, 0);
  analogWrite(m2_pin2, 0);

  while(1);

}

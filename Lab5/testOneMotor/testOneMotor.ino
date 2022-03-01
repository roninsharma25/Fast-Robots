// Motor 1

int pin1 = A2;
int pin2 = A3;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Forward
  analogWrite(pin1, 200);
  analogWrite(pin2, 200);

  delay(500);

  // Stop
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);

  delay(500);

}

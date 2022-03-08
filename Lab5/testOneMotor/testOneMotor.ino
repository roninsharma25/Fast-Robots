// Motor 1

int pin1 = 2;
int pin2 = 3;

void setup() {
  Serial.begin(9600);
  
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

void loop() {
  // Forward
  analogWrite(pin1, 100);
  analogWrite(pin2, 0);

  delay(2000);

  // Stop
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);

  while(1);
}

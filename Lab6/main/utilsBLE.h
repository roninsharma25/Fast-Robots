
enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    MOVE_FORWARD,
    STOP_ROBOT,
    GET_IMU,
    UPDATE_PID
};

void moveForwardCase(int speed1, int speed2, int forward) {
  
  if (forward == 0) {

    Serial.print("forward");
    moveForward(speed1, speed2);
    
  } else {
    
    Serial.print("backward");
    moveBackward(speed1, speed2);

  }
}

void delay_(int time_, BLEDevice ble) {
  unsigned long startTime = millis();
  while (millis() - startTime < time_) {
     int check = ble.connected();
  }
}

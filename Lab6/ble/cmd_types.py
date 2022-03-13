from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    MOVE_FORWARD = 6
    STOP_ROBOT = 7
    GET_IMU = 8
    UPDATE_PID = 9

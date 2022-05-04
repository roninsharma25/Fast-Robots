from enum import Enum

class CMD(Enum):
    PING = 0
    MOVE_FORWARD = 1
    STOP_ROBOT = 2
    UPDATE_PID = 3
    TURN = 4
    TURN_360 = 5

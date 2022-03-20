
class RobotControl():
    # Initialize Function
    def __init__(self, ble):
        self.ble = ble
        
        # A variable to store the latest sensor value
        self.latest_tof_front_reading = None
        self.latest_tof_side_reading = None
        
        # A list to store the history of all the sensor values
        # Each item in the list is a tuple (value, time)
        # WARNING: The list could grow really fast; you need to deal with this accordingly.
        self.tof_readings = []
        self.tof2_readings = []
        
        self.imu_readings = []
        
        self.motor_readings = []
        
        # A variable to store the latest imu reading
        self.latest_imu_reading = None
        
        # Activate notifications (if required)
        self.setup_notify()
    
    # A function to activate various notifications (if required)
    def setup_notify(self):
        # Code to setup various notify events
        # Ex:
        # ble.start_notify(ble.uuid['RX_TOF2'], self.tof_callback_handler)
        # ble.start_notify(ble.uuid['RX_FANCY_SENSOR'], self.fancy_sensor_callback_handler)
        self.ble.start_notify(self.ble.uuid['RX_TOF1'], self.tof_callback_handler)
        self.ble.start_notify(self.ble.uuid['RX_TOF2'], self.tof2_callback_handler)
        self.ble.start_notify(self.ble.uuid['RX_IMU'], self.imu_callback_handler)
        self.ble.start_notify(self.ble.uuid['RX_MOTOR'], self.motor_callback_handler)
    
    def stop_notify(self, sensor):
        self.ble.stop_notify(ble.uuid[f'RX_{sensor}'])
        
    # An example function callback handler for storing the history of the tof sensor
    # Your callback handlers should perform minimal processing!
    # Do not add a receive_* function inside the callback handler, it defeats the purpose of BLE notify
    def tof_callback_handler(self, uuid, byte_array):
        # Append a tuple (value, time) to a list
        self.tof_readings.append( ( self.ble.bytearray_to_float(byte_array), time.time() ) )
        #self.update_tof_readings()
    
    def tof2_callback_handler(self, uuid, byte_array):
        # Append a tuple (value, time) to a list
        self.tof2_readings.append( ( self.ble.bytearray_to_float(byte_array), time.time() ) )
        #self.update_tof2_readings()
    
    def imu_callback_handler(self, uuid, byte_array):
        self.imu_readings.append( ( self.ble.bytearray_to_float(byte_array), time.time() ) )
        #self.update_imu_readings()
    
    def motor_callback_handler(self, uuid, byte_array):
        self.motor_readings.append( ( self.ble.bytearray_to_float(byte_array), time.time() ) )
    
    def update_imu_readings(self):
        if len(self.imu_readings) > 10000:
            self.imu_readings = self.imu_readings[-10000:]
    
    def update_tof_readings(self):
        if len(self.tof_readings) > 10000:
            self.tof_readings = self.tof_readings[-10000:]
    
    def update_tof2_readings(self):
        if len(self.tof2_readings) > 10000:
            self.tof2_readings = self.tof2_readings[-10000:]
    
    # An example function to fetch the front TOF sensor reading
    # Here we assume RX_TOF1 is a valid UUID defined in connection.yaml and
    # in the Arduino code as well
    def get_front_tof(self):
        self.latest_tof_front_reading = self.ble.receive_float(self.ble.uuid['RX_TOF2'])
        print(self.latest_tof_front_reading)
    
    def get_side_tof(self):
        self.latest_tof_side_reading = self.ble.receive_float(self.ble.uuid['RX_TOF1'])
        print(self.latest_tof_side_reading)
    
    # An example function to fetch the IMU readings as a string
    # Here we assume RX_IMU is a valid UUID defined in connection.yaml and
    # in the Arduino code as well
    def get_imu(self):
        self.ble.send_command(CMD.GET_IMU, '')
        self.latest_imu_reading = self.ble.receive_float(self.ble.uuid['RX_IMU'])
        
        print(self.latest_imu_reading)
    
    # A function to instruct the robot to move forward
    def move_forward(self, speed, forward, doPID = 0):
        # Code to move forward
        # Ex: 
        # Here we assume the command is defined in cmd_types.py and 
        # the Artemis is programmed to handle it accordingly
        # ble.send_command(CMD.MOVE_FORWARD, speed)
        
        """
        speed: two element list ([motor1Speed, motor2Speed])
        time: duration of robot movement in seconds
        forward:
        doPID: 0 when PID should be done and 1 otherwise
        """
        self.tof_readings = []
        self.tof2_readings = []
        
        self.ble.send_command(CMD.MOVE_FORWARD, f'{speed[0]}|{speed[1]}|{forward}|{doPID}')
    
    # A function to stop robot motion
    def stop(self):
        # Code to stop robot motion
        self.ble.send_command(CMD.STOP_ROBOT, '')
    
    # A function to instruct the robot to update PID constants
    def updatePID(self, setpoint, k_p, k_i, k_d):
        self.ble.send_command(CMD.UPDATE_PID, f'{setpoint}|{k_p}|{k_i}|{k_d}')


 
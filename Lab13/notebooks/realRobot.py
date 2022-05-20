import os
import pathlib

class RealRobot:
    """A class to interact with the real robot
    """
    def __init__(self, commander, ble):
        # Load world config
        self.world_config = os.path.join(str(pathlib.Path(os.getcwd()).parent), "config", "world.yaml")
        
        self.config_params = load_config_params(self.world_config)
        
        # Commander to commuincate with the Plotter process
        # Used by the Localization module to plot odom and belief
        self.cmdr = commander

        # ArtemisBLEController to communicate with the Robot
        self.ble = ble
        
        # Robot Control attribute
        self.rc = RobotControl(ble)

    def get_pose(self):
        """Get robot pose based on odometry
        
        Returns:
            current_odom -- Odometry Pose (meters, meters, degrees)
        """
        raise NotImplementedError("get_pose is not implemented")

    async def perform_observation_loop(self, rot_vel=120):
        """Perform the observation loop behavior on the real robot, where the robot does  
        a 360 degree turn in place while collecting equidistant (in the angular space) sensor
        readings, with the first sensor reading taken at the robot's current heading. 
        The number of sensor readings depends on "observations_count"(=18) defined in world.yaml.
        
        Keyword arguments:
            rot_vel -- (Optional) Angular Velocity for loop (degrees/second)
                        Do not remove this parameter from the function definition, even if you don't use it.
        Returns:
            sensor_ranges   -- A column numpy array of the range values (meters)
            sensor_bearings -- A column numpy array of the bearings at which the sensor readings were taken (degrees)
                               The bearing values are not used in the Localization module, so you may return a empty numpy array
        """
        #self.rc.pingRobot(clear = True) # clear attributes
        #self.rc.turn360(forwardSpeed = 140, backwardSpeed = 140, dir_ = 0)
        #await asyncio.sleep(5)
        
        sensorVals = np.array(x).reshape(18, 1)/1000#np.array(loc_0_3[0]).reshape(18, 1)
        #sensorVals = np.ones(18).reshape(18, 1)
        #sensorVals = np.ones(18).reshape(18, 1)
        gyroVals = np.array([2974.44995117, 2974.45556641, 2993.55810547, 3012.73193359, 3031.74584961, 
                             3050.94506836, 3070.16479492, 3089.19018555, 3108.37109375, 3127.76220703,
                             3147.09936523, 3166.15039062, 3185.69360352, 3204.91992188, 3224.35595703,
                             3243.38232422, 3262.41137695, 3281.63598633]).reshape(18, 1)
        
        
        #sensorVals = np.array( [ val[0] / 1000 for val in self.rc.tof2_readings] ).reshape(18, 1)
        #gyroVals = np.array( [ val[0] for val in self.rc.imu_readings] ).reshape(18, 1)
        
        print(sensorVals)
        print(gyroVals)
        
        # RETURN DISTANCES AND ANGLES
        return sensorVals, gyroVals
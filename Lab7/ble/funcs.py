

def distToVelocity(data):
    startTime = data[0][1]
    
    prevDist = data[0][0]
    prevTime = startTime
    
    velocities = [0]
    times = [0]
    
    for val in data[1:]:
        currDist = val[0]
        currTime = val[1]
        velocities.append( (currDist - prevDist) / (currTime - prevTime) ) # v = d / t
        times.append( currTime - startTime )
        
        # update previous data
        prevDist = currDist
        prevTime = currTime
    
    return [velocities, times]


def dataForPlot(data):
    sensorInfo = []
    times = []
    
    for val in data:
        sensorInfo.append(val[0])
        times.append(val[1])
    
    return [ sensorInfo, times ]


def processRunData(file):
    with open(file, 'r') as f:
        x = f.read().splitlines()
    
    comma = x[0].index(",")
    oldSensorValue = float(x[0][1:comma])
    oldTimeValue = startTimeValue = float(x[0][comma+2:-1])
    
    outputData = [ [oldSensorValue, 0] ] # [ [sensor value, time value]]

    for val in x[1:]:

        comma = val.index(",")
        newSensorValue = float(val[1:comma])
        newTimeValue = float(val[comma+2:-1]) - startTimeValue

        outputData.append( [ newSensorValue, newTimeValue ] )
    
    return outputData


def storeData(data, file):
    #file = 'trial102_motor.txt'
    with open(file, 'w') as f:
        for item in data:
            f.write(str(item))
            f.write("\n")


def processRunData(file):
    with open(file, 'r') as f:
        x = f.read().splitlines()
    
    calculate = False
    comma = x[0].index(",")
    oldSensorValue = float(x[0][1:comma])
    oldTimeValue = float(x[0][comma+2:-1])

    for val in x[1:]:

        comma = val.index(",")
        newSensorValue = float(val[1:comma])
        newTimeValue = float(val[comma+2:-1])

        #print(newTimeValue-oldTimeValue)

        if (newTimeValue - oldTimeValue) > 0.01:
            timeDelta = newTimeValue - oldTimeValue
            sensorDelta = newSensorValue - oldSensorValue
            
            speed = sensorDelta/(timeDelta * 1000) # convert mm/s to m/s
            if speed > 0:
                print(f'{speed} m/s')

            oldTimeValue = newTimeValue
            oldSensorValue = newSensorValue

def plot(x):
    start_time = x[0][1]
    plt.plot([(x[i][1] - start_time) for i in range(len(x))], [x[i][0] for i in range(len(x))])
    plt.title('Front ToF Sensor Reading (mm) vs. Time (msec)')
    plt.xlim([0, 0.032])



def calcRiseTime(data, start, max_, percent = 90):
    startTime = data[0][1]
    topValue = max_ * percent / 100
    for val in data: # data format - [ (motorPWM1, time1), (motorPWM2, time2) ]
        # Get the PWM signal
        if val[0] >= topValue:
            return val[1] - startTime
        
        
def performKF(data, A, B, n = 2):
    x = np.array([[sensorVal],[0]])
    
    Ad = np.eye(n) + Delta_T * A  # n is the dimension of your state space 
    Bd = Delta_t * B
    
    # Use numpy linspace and interp to make data the same size (if needed)
    
    # loop over data and perform KF
    for val in data:
        sensorVal = val[0]
        timeVal = val[1]
        
        # STILL NEED TO ADD: sigma, y
        kf(timeVal, sigma, sensorVal, y, Ad, Bd)


# C = 1 -> measure positive distance from the wall (initial state)
def kf(mu, sigma, u, y, A, B, C = np.array([[1, 0]])):

    mu_p = A.dot(mu) + B.dot(u) 
    sigma_p = A.dot(sigma.dot(A.transpose())) + Sigma_u
    
    y_m = y-C.dot(mu_p)
    sigma_m = C.dot(sigma_p.dot(C.transpose())) + Sigma_n
    kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))

    mu = mu_p + kkf_gain.dot(y_m)    
    sigma=(np.eye(3)-kkf_gain.dot(C)).dot(sigma_p)

    return mu,sigma            

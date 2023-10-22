from database import database
from threading import Thread
import serial
from time import sleep as delay
import math
# import matplotlib.pyplot as plt


## ---------- Kalman variables & functions
prevYaw = 0
KalmanAnglePitch = 0
KalmanAngleYaw = 0
KalmanUncertaintyAnglePitch = 2 * 2
KalmanUncertaintyAngleYaw = 2 * 2
KalmanDOutput = [0, 0]

def kalman_1d(KalmanState, KalmanUncertainty, KalmanMeasurement):
    
    KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3)
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    KalmanDOutput[0] = KalmanState
    KalmanDOutput[1] = KalmanUncertainty
    
    # print(f'Predict: {KalmanState}, Measured: {KalmanMeasurement}')
    return KalmanState, KalmanMeasurement
    
def KalmanCal(show):
    global KalmanAngleYaw, KalmanAnglePitch, KalmanUncertaintyAngleYaw, KalmanUncertaintyAnglePitch, KalmanDOutput
    Ax = db.acc.x
    Ay = db.acc.y
    Az = db.acc.z
    Gx = db.gyro.x
    Gy = db.gyro.y
    Gz = db.gyro.z

    AnglePitch = math.atan(Ay / math.sqrt(Ax * Ax + Az * Az)) * 1 / (3.142 / 180);
    # AngleRoll = -math.atan(Ax / math.sqrt(Ay * Ay + Az * Az)) * 1 / (3.142 / 180);
    AngleYaw = math.atan(Az / math.sqrt(Az * Az + Az * Az)) * 1 / (3.142 / 180);
     

    # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
    if show:
        print('G: x-%.2f y-%.2f z-%.2f\t\tA: x-%.2f y-%.2f z-%.2f' % (Gx, Gy, Gz, Ax, Ay, Az))

    # Kalman filter for pitch
    timeConst = 0.2 
    pitchVal = kalman_1d(
        KalmanAnglePitch + timeConst * Gx, 
        KalmanUncertaintyAnglePitch + timeConst * timeConst * 16, 
        AnglePitch);
    KalmanAnglePitch = KalmanDOutput[0];
    KalmanUncertaintyAnglePitch = KalmanDOutput[1];
    
    # Kalman filter for yaw
    def kalman_yaw(KalmanState, KalmanUncertainty, KalmanInput):
    KalmanState = KalmanState + timeConstYaw * KalmanInput
    KalmanUncertainty = KalmanUncertainty + timeConstYaw * timeConstYaw * 16
    
    KalmanGain = KalmanUncertainty / (KalmanUncertainty + 16)
    KalmanState = KalmanState + KalmanGain * (0 - KalmanState)  # Assuming yaw measurement is 0 for simplicity
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    
    return KalmanState, KalmanUncertainty

    # Update yaw using Kalman filter
    pitchVal = kalman_yaw(KalmanAngleYaw, KalmanUncertaintyAngleYaw, Gz)
    KalmanAngleYaw = pitchVal[0]
    KalmanUncertaintyAngleYaw = pitchVal[1]
    
    
    print(f'localPitch: {AnglePitch}, localPitch: {AngleYaw}')
    return pitchVal, YawVal
        

## ---------- Serial command functions
def transmit(port, txt):
    txt += '\n'
    ser.write(bytes(txt, 'utf-8'))


## ---------- Variables
db = database('Whaly')
errCnt = 0
sentRoutine = 0
criticalAngle = 20


## ---------- Main program
if __name__ == '__main__':
    # Start multiprocess
    thread1 = Thread(target= db.get)
    thread1.start()
    
    # Initiate serial communication
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.reset_input_buffer()
    
            
        
    # Main program
    I = 0
    D = 0
    prev = 0
    while True:
        # Get data from database
        db.get()
        print(db.yaw)
        
        # Execute kalman filter
        KalmanCal(False)
        
        # Set value for PID
        Kp = 1.2
        Kd = 0.6
        baseSpeed = 60
        current = db.yaw
        D = current - prev
        prev = current
        
        # PID calculaiton & motor speed set
        P = db.yaw * Kp + D * Kdjk
        transmit(ser, f'{baseSpeed + P}_{baseSpeed - P}')
        

        
        # Check the whether the robot is climbing
        if abs(KalmanAnglePitch) > criticalAngle:
            errCnt+= 1  
        
        # if it's climbing more than 2 seconds
        if errCnt >= 20:
            errCnt = 0
            
            print('Stop')
            ser.write(bytes(f"0_0\n", 'utf-8'))
            delay(2)
            
            print('Reverse')
            ser.write(bytes(f"-50_-50\n", 'utf-8'))
            delay(5)
            
            print('Stop')
            ser.write(bytes(f"0_0\n", 'utf-8'))
            delay(15)
            
            
        # set data sending routine
        elif sentRoutine < 5:
            sentRoutine+= 1
        else:
            sentRoutine = 0
            ser.write(bytes(f"60_60\n", 'utf-8'))
            print('sent')
        
        
        
        delay(0.1)
    
    
    # Test Route PID
    '''
    I = 0
    D = 0
    prev = 0
    while True:
        db.get()
        print(db.yaw)
        Kp = 1
        Ki = 0
        Kd = 0.5
        baseSpeed = 40
        current = db.yaw
        D = current - prev
        prev = current
        
        P = db.yaw * Kp + D * Kd
        transmit(ser, f'{baseSpeed + P}_{baseSpeed - P}')
        KalmanCal(False)
        
        delay(0.1)

    '''  
    
    
    # Test incline
    '''
    while True:
        
        db.get()
        # db.showAll()
        KalmanCal(False)
        
        
        # Minimamum track damage
        if abs(KalmanAnglePitch) > criticalAngle:
            errCnt+= 1  
        
        if errCnt >= 5:
            errCnt = 0
            
            print('Stop')
            ser.write(bytes(f"0_0\n", 'utf-8'))
            delay(2)
            
            print('Reverse')
            ser.write(bytes(f"-50_-50\n", 'utf-8'))
            delay(5)
            
            print('Stop')
            ser.write(bytes(f"0_0\n", 'utf-8'))
            delay(15)
            
            
            # delay(10)
        elif sentRoutine < 5:
            sentRoutine+= 1
        else:
            sentRoutine = 0
            ser.write(bytes(f"60_60\n", 'utf-8'))
            print('sent')
        
        delay(0.1)
    '''
        
        
        
    # Test Routine turm 90 degree
    '''
    while True:
        transmit(ser, '0_0')
        _in = input('waiting...')
        db.get()
        print(db.yaw)
        
        prev = db.yaw
        
        transmit(ser, '25_-25')
        while abs(db.yaw - prev) < 80:
        # while abs(db.yaw) < 75:
            db.get()
            print(db.yaw)
            delay(0.1)
        transmit(ser, '0_0')
        # db.showAll()
        # KalmanCal(False)
        
        # _in = input('Enter Command: ')
        
        
        
        # transmit(ser, '60_-60')
        # delay(1.8)
        # transmit(ser, '0_0')
        
        
        delay(1)
        '''
        
    
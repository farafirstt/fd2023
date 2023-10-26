from database import database
from threading import Thread
import serial
from time import sleep as delay
import math
import rpi.GPIO as GPIO
import time
import keyboard
# import matplotlib.pyplot as plt


## ---------- Kalman variables & functions
prevYaw = 0
KalmanAnglePitch = 0
KalmanAngleYaw = 0
KalmanUncertaintyAnglePitch = 2 * 2
KalmanUncertaintyAngleYaw = 2 * 2
KalmanDOutput = [0, 0]

def kalman_1d(KalmanState, KalmanUncertainty, KalmanMeasurement):
    # KalmanState = predicted value
    # KalmanUncertainty = predicted uncertainty
    # KalmanMeasurement = measured value
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
    AngleYaw = math.atan(-Gx / math.sqrt(Gy * Gy + Gz * Gz)) * 1 / (3.142 / 180)

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
    timeConst = 0.2
    yawVal = kalman_1d(
        KalmanAngleYaw + timeConst * Gz, 
        KalmanUncertaintyAngleYaw + timeConst * timeConst * 16, 
        AngleYaw);
    KalmanAngleYaw = KalmanDOutput[0];
    KalmanUncertaintyAngleYaw = KalmanDOutput[1];
    
    print(f'localPitch: {AnglePitch}')
    print(f'localYaw: {AngleYaw}')
    return pitchVal, yawVal

## ---------- Serial command functions
def transmit(port, txt):
    txt += '\n'
    ser.write(bytes(txt, 'utf-8'))


## ---------- Variables
db = database('Whaly')
errCnt = 0
sentRoutine = 0
criticalAngle = 20

# grid = size of car
tpg = 10 # t for 1 size of grid

# ---------- Set GPIO pin for ultrasonic sensor
# Note: if echo pin is the same pin, use the same variable -> echo
front1_trig = 0
front1_echo = 0
front2_trig = 0
front2_echo = 0
front3_trig = 0
front3_echo = 0

left1_trig = 0
left1_echo = 0
left2_trig = 0
left2_echo = 0

right1_trig = 0
right1_echo = 0
right2_trig = 0
right2_echo = 0

bottom1_trig = 0
bottom1_echo = 0
bottom2_trig = 0
bottom2_echo = 0

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(front1_trig, GPIO.OUT)
GPIO.setup(front1_echo, GPIO.IN)
GPIO.setup(front2_trig, GPIO.OUT)
GPIO.setup(front2_echo, GPIO.IN)
GPIO.setup(front3_trig, GPIO.OUT)
GPIO.setup(front3_echo, GPIO.IN)

GPIO.setup(left1_trig, GPIO.OUT)
GPIO.setup(left1_echo, GPIO.IN)
GPIO.setup(left2_trig, GPIO.OUT)
GPIO.setup(left2_echo, GPIO.IN)

GPIO.setup(right1_trig, GPIO.OUT)
GPIO.setup(right1_echo, GPIO.IN)
GPIO.setup(right2_trig, GPIO.OUT)
GPIO.setup(right2_echo, GPIO.IN)

GPIO.setup(bottom1_trig, GPIO.OUT)
GPIO.setup(bottom1_echo, GPIO.IN)
GPIO.setup(bottom2_trig, GPIO.OUT)
GPIO.setup(bottom2_echo, GPIO.IN)

# ultrasonic sensor function
def distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.5)
    
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    return distance

# Set up ultrasonic sensor
frontsensor = [distance(front1_trig, front1_echo), distance(front2_trig, front2_echo), distance(front3_trig, front3_echo)]
leftsensor = [distance(left1_trig, left1_echo), distance(left2_trig, left2_echo)]
rightsensor = [distance(right1_trig, right1_echo), distance(right2_trig, right2_echo)]

threshold = 10 # threshold for ultrasonic

# ---------- Function for motor control
def stop():
    ser.write(bytes(f"0_0\n", 'utf-8'))
    delay(2)

def forward(time):
    ser.write(bytes(f"60_60\n", 'utf-8'))
    delay(time)

def slowForward(time):
    ser.write(bytes(f"30_30\n", 'utf-8'))
    delay(time)

def backward(time):
    ser.write(bytes(f"-60_-60\n", 'utf-8'))
    delay(time)

def left():
    db.get()
    prevYaw = db.yaw
    ser.write(bytes(f"-60_60\n", 'utf-8'))
    delay(1)
    dYaw = db.yaw - prevYaw
    while abs(dYaw) < 90:
        dYaw = db.yaw - prevYaw
        ser.write(bytes(f"-60_60\n", 'utf-8'))
        delay(0.1)
    stop()

def right():
    db.get()
    prevYaw = db.yaw
    ser.write(bytes(f"60_-60\n", 'utf-8'))
    delay(1)
    dYaw = db.yaw - prevYaw
    while abs(dYaw) < 90:
        dYaw = db.yaw - prevYaw
        ser.write(bytes(f"60_-60\n", 'utf-8'))
        delay(0.1)

def left45():
    db.get()
    prevYaw = db.yaw
    ser.write(bytes(f"-60_60\n", 'utf-8'))
    delay(1)
    dYaw = db.yaw - prevYaw
    while abs(dYaw) < 45:
        dYaw = db.yaw - prevYaw
        ser.write(bytes(f"-60_60\n", 'utf-8'))
        delay(0.1)
    stop()

def right45():
    db.get()
    prevYaw = db.yaw
    ser.write(bytes(f"60_-60\n", 'utf-8'))
    delay(1)
    dYaw = db.yaw - prevYaw
    while abs(dYaw) < 45:
        dYaw = db.yaw - prevYaw
        ser.write(bytes(f"60_-60\n", 'utf-8'))
        delay(0.1)
    stop()
    
def uturn():
    print('Stop')
    stop()
    print('Backward')
    backward(2)
    print('Right')
    right()
    print('Forward')
    forward(tpg)
    print('Right')
    right()

def obstacle():
    condition =''
    if frontsensor[1] < threshold and (frontsensor[0] < threshold or leftsensor[0] < threshold or leftsensor[1] < threshold):
        condition = 'Right45'
    elif frontsensor[1] < threshold and (frontsensor[2] < threshold or rightsensor[0] < threshold or rightsensor[1] < threshold):
        condition = 'Left45'
    elif any(i < threshold for i in leftsensor) and any(i < threshold for i in rightsensor):
        pass
    elif frontsensor[0] < threshold or leftsensor[0] < threshold or leftsensor[1] < threshold:
        condition = 'Right45'
    elif frontsensor[2] < threshold or rightsensor[0] < threshold or rightsensor[1] < threshold:
        condition = 'Left45'
    elif frontsensor[1] < threshold:
        condition = 'Right45'
    
    return condition

## ---------- Manual control
def manual_mode(state):
    while state:
        try:
            if keyboard.is_pressed('w'):
                forward(0.2)
            elif keyboard.is_pressed('s'):
                backward(0.2)
            elif keyboard.is_pressed('e'):
                left()
            elif keyboard.is_pressed('q'):
                right()
        except KeyboardInterrupt:
            break


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
        P = db.yaw * Kp + D * Kd
        transmit(ser, f'{baseSpeed + P}_{baseSpeed - P}')

        # set rounds
        if obstacle:
            print("Backward")
            backward()
            time.sleep(1)
            print(obstacle)
            if obstacle == 'Right45':
                right45()
            elif obstacle == 'Left45':
                left45()

        #ser data sending routine for direction control
        elif sentDir < 100:
            sentDir+= 1
        else:
            sentDir = 0
            print("Uturn")

        
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
            
            
        # set data sending routine for speed control
        elif sentRoutine < 5:
            sentRoutine+= 1
        else:
            sentRoutine = 0
            ser.write(bytes(f"60_60\n", 'utf-8'))
            print('sent')
        
        # manual control
        if keyboard.is_pressed('m'):
            manual_mode(True)
            manual_mode(False)
        
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
        
        '''  
        # ultrasonic sensor
        front1 = distance(front1_trig, front1_echo)
        front2 = distance(front2_trig, front2_echo)
        front3 = distance(front3_trig, front3_echo)
        left1 = distance(left1_trig, left1_echo)
        left2 = distance(left2_trig, left2_echo)
        right1 = distance(right1_trig, right1_echo)
        right2 = distance(right2_trig, right2_echo)
        bottom1 = distance(bottom1_trig, bottom1_echo)
        bottom2 = distance(bottom2_trig, bottom2_echo)

        mindistance = 10
        t = 5 # t that robot move 1 grid

        # front obstacle
        if (front1 < mindistance and front2 < mindistance) or (front1 < mindistance):
            print('Stop')
            slowForward()
            print('Backward')
            print('Right')
            print('Forward until all left clear')
            print('Left')
            print('Forward for t second')
            print('Forward until all left clear')
            print('Left')
            print('Forward until all left found') # can be changed to 'for t second'
            print('Right') # back to origin track
            normalForward()
            print('Forward')

        if (front3 < mindistance and front2 < mindistance) or (front3 < mindistance):
            print('Stop')
            slowForward()
            print('Backward')
            print('Left')
            print('Forward until all right clear')
            print('Right')
            print('Forward for t second')
            print('Forward until all right clear')
            print('Right')
            print('Forward until all right found') # can be changed to 'for t second'
            print('Left') # back to origin track
            normalForward()
            print('Forward')

        if front1 < mindistance and front2 < mindistance and front3 < mindistance:
            print('Stop')
            slowForward()
            print('Backward')
            print('Left')
            print('Forward until right clear')
            print('Right')
            print('Forward for t second')
            print('Forward until right clear')
            print('Right')
            print('Forward until right found') # can be changed to 'for t second'
            print('Left') # back to origin track
            normalForward()
            print('Forward')

        ## What if face dead end???
        # distance between left and right is not enough to detect narrow track
        # but it is too naroow to turn around
        
        '''
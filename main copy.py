from database import database
from threading import Thread
from time import sleep as delay
import math
# import matplotlib.pyplot as plt
#11112222

## ---------- Kalman variables & functions
KalmanAngleRoll = 0
KalmanAnglePitch = 0
KalmanUncertaintyAngleRoll = 2 * 2
KalmanUncertaintyAnglePitch = 2 * 2
KalmanDOutput = [0, 0]

def kalman_1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement):
    timeConst = 0.2
    KalmanState = KalmanState + timeConst * KalmanInput
    KalmanUncertainty = KalmanUncertainty + timeConst * timeConst * 4 * 4
    
    KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3)
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    KalmanDOutput[0] = KalmanState
    KalmanDOutput[1] = KalmanUncertainty
    
    # print(f'Predict: {KalmanState}, Measured: {KalmanMeasurement}')
    return KalmanState, KalmanMeasurement
    
def KalmanCal(show):
    global KalmanAngleRoll, KalmanAnglePitch, KalmanUncertaintyAngleRoll, KalmanUncertaintyAnglePitch, KalmanDOutput
    Ax = db.acc.x
    Ay = db.acc.y
    Az = db.acc.z
    Gx = db.gyro.x
    Gy = db.gyro.y
    Gz = db.gyro.z

    AngleRoll = math.atan(Ay / math.sqrt(Ax * Ax + Az * Az)) * 1 / (3.142 / 180);
    AnglePitch = -math.atan(Ax / math.sqrt(Ay * Ay + Az * Az)) * 1 / (3.142 / 180);
    AngleYaw = math.atan(Az / math.sqrt(Az * Az + Az * Az)) * 1 / (3.142 / 180);
       

    # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
    if show:
        print('G: x-%.2f y-%.2f z-%.2f\t\tA: x-%.2f y-%.2f z-%.2f' % (Gx, Gy, Gz, Ax, Ay, Az))

    rollVal = kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Gx, AngleRoll);
    KalmanAngleRoll = KalmanDOutput[0];
    KalmanUncertaintyAngleRoll = KalmanDOutput[1];
    
    pitchVal = kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Gy, AnglePitch);
    KalmanAnglePitch = KalmanDOutput[0];
    KalmanUncertaintyAnglePitch = KalmanDOutput[1];
    
    YawVal = kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, Gz, AngleYaw);
    KalmanAngleYaw = KalmanDOutput[0];
    KalmanUncertaintyAngleYaw = KalmanDOutput[1];    

    print(f'localRow: {AngleRoll}, localPitch: {AnglePitch}, localYaw: {AngleYaw}')
    return rollVal, pitchVal ,YawVal
        


## ---------- Variables
db = database('Prachya')


## ---------- Main program
if __name__ == '__main__':
    # Start multiprocess
    thread1 = Thread(target= db.get)
    thread1.start()
    
    KalmanState=0
    while True:
        
        db.get()
        # db.showAll()
        KalmanCal(False)
        
        
        delay(0.1)
        









    



# Kalman filter test
# Add Yaw

import math

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
    Ax = 0.16496
    Ay = -0.24942
    Az = -0.92068
    Gx = -0.26647
    Gy = 0.26076
    Gz = 0.50096

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

print(KalmanCal(False))
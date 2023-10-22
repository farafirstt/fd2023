from time import sleep          #import
import math

# Kalman variables
KalmanAngleRoll = 0
PKalmanAngleitch = 0
YKalmanAngleaw = 0
KalmanUncertaintyAngleRoll = 2 * 2
KalmanUncertaintyAnglePitch = 2 * 2
KalmanUncertaintyAngleYaw = 2 * 2
KalmanDOutput = [0, 0]

n = 0
count = 0
errorTime = 2
step = errorTime/0.3;

criticalIncline = -20


def kalman_1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement):
    KalmanState = KalmanState + 0.004 * KalmanInput
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4
    
    KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3)
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    KalmanDOutput[0] = KalmanState
    KalmanDOutput[1] = KalmanUncertainty
    


if __name__ == '__main__':

    AngleRoll = math.atan(Ay / math.sqrt(Ax * Ax + Az * Az)) * 1 / (3.142 / 180);
    AnglePitch = -math.atan(Ax / math.sqrt(Ay * Ay + Az * Az)) * 1 / (3.142 / 180);

    #
    AngleYaw = math.atan(Az / math.sqrt(Az * Az + Az * Az)) * 1 / (3.142 / 180);
        

    # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
    print('G: x-%.2f y-%.2f z-%.2f\t\tA: x-%.2f y-%.2f z-%.2f' % (Gx, Gy, Gz, Ax, Ay, Az))

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Gx, AngleRoll);
    KalmanAngleRoll = KalmanDOutput[0];
    KalmanUncertaintyAngleRoll = KalmanDOutput[1];
    
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Gy, AnglePitch);
    KalmanAnglePitch = KalmanDOutput[0];
    KalmanUncertaintyAnglePitch = KalmanDOutput[1];
    # print('Roll: %.2f, Pitch: %.2f' % (AngleRoll, AnglePitch))    
    kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, Gz, AngleYaw);
    KalmanAngleYaw = KalmanDOutput[0];
    KalmanUncertaintyAngleYaw = KalmanDOutput[1];    

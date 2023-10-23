# This code is for testing the condition of the robot
# It will be used for the obstacle avoidance

frontsensor = [0, 10, 0]
leftsensor = [10, 10]
rightsensor = [10, 10]

threshold = 5 # threshold for ultrasonic

def obstacle():
    condition =''
    if (leftsensor[0] < threshold or leftsensor[1] < threshold) and (rightsensor[0] < threshold or rightsensor[1] < threshold):
        condition = 'Forward'
    elif frontsensor[1] < threshold and (frontsensor[0] < threshold or leftsensor[0] < threshold or leftsensor[1] < threshold):
        condition = 'Right45'
    elif frontsensor[1] < threshold and (frontsensor[2] < threshold or rightsensor[0] < threshold or rightsensor[1] < threshold):
        condition = 'Left45'
    elif frontsensor[0] < threshold or leftsensor[0] < threshold or leftsensor[1] < threshold:
        condition = 'Right45'
    elif frontsensor[2] < threshold or rightsensor[0] < threshold or rightsensor[1] < threshold:
        condition = 'Left45'
    elif frontsensor[1] < threshold:
        condition = 'Right45'

    return condition

print(obstacle())

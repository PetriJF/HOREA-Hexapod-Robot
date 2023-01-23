from time import *
from dataclasses import dataclass
import numpy as np
from adafruit_servokit import ServoKit
from math import degrees
kit = ServoKit(channels=16)

@dataclass
class Point:
    x: float
    y: float    
    z: float

@dataclass
class Angles:
    hip: float
    shoulder: float    # height
    knee: float

# Constants
COXA_LEN = 71.5
FEMUR_LEN = 100.02
FOOT_HEIGHT = 0
TIBIA_LEN = 150 + FOOT_HEIGHT
GAIT_ALTITUDE = 90
GAIT_WIDTH = 180
BASE_ALTITUDE = 40
HIP_ORIGIN = Point(0,BASE_ALTITUDE,0)

# Initialize
#kit.servo[0].angle = 90
#kit.servo[1].angle = 90
#kit.servo[2].angle = 90
#sleep(3)


def getLegAngles(point):
    L = np.sqrt((point.x - HIP_ORIGIN.x)**2 + (point.z - HIP_ORIGIN.z)**2)
    D = np.sqrt(L*L + (point.y - HIP_ORIGIN.y)**2)
    print(L, D)

    alpha = np.arctan(np.absolute(point.y - HIP_ORIGIN.y) / (L))
    beta = np.arccos((D * D + FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN) / (2 * FEMUR_LEN * D))
    gamma = np.arccos((FEMUR_LEN *FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - D * D) / (2 * FEMUR_LEN * TIBIA_LEN))
    print(alpha, beta, gamma, np.degrees(alpha), np.degrees(beta), np.degrees(beta-alpha))

    tetha = np.degrees(np.arctan((point.z - HIP_ORIGIN.z) / (point.x - HIP_ORIGIN.x)))
    tetha1 = 180 - np.degrees(beta + alpha)
    tetha2 = 180 - np.degrees(gamma)
    #tetha = np.degrees(np.arctan((point.z - HIP_ORIGIN.z) / (point.x - HIP_ORIGIN.x)))
    #tetha1 = np.degrees(np.pi / 2 - (np.arccos((TIBIA_LEN*TIBIA_LEN - FEMUR_LEN*FEMUR_LEN - L2*L2) / (-2 * FEMUR_LEN * L2)) - alpha))
    #tetha2 =  np.degrees(np.pi -np.arccos((L2*L2 - FEMUR_LEN*FEMUR_LEN - TIBIA_LEN*TIBIA_LEN) / (-2 * FEMUR_LEN * TIBIA_LEN)))

    return Angles(round(tetha, 2), round(tetha1, 2), round(tetha2, 2))

def setMotorAngles(angles):
    print(angles)
    kit.servo[0].angle = 90 + angles.hip
    kit.servo[1].angle = angles.shoulder
    kit.servo[2].angle = angles.knee
    sleep(1)

## MAIN CODE

while True:
    one = Point(140, 0, 0)
    two = Point(180, GAIT_ALTITUDE, 0)
    three = Point(240, 0, 0)
    
    setMotorAngles(getLegAngles(one))

    setMotorAngles(getLegAngles(two))

    setMotorAngles(getLegAngles(three))


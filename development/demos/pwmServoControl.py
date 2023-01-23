from time import *
from dataclasses import dataclass
import numpy as np
from adafruit_servokit import ServoKit
from math import degrees
kit = ServoKit(channels=16)

## MAIN CODE

kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[0].angle = 0

sleep(1)
for pos in range(0, 180):
    kit.servo[0].angle = pos
    sleep(0.05)


for pos in range(0, 180):
    kit.servo[0].angle = 180 - pos
    sleep(0.05)




    


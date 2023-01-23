from time import *
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)


while True:
    sleep(2)
    kit.servo[0].angle = 90
    kit.servo[1].angle = 90
    kit.servo[2].angle = 0
    #sleep(2)
    #kit.servo[0].angle = 0
    #kit.servo[1].angle = 63
    #kit.servo[2].angle = 0
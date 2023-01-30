#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import *
from adafruit_servokit import ServoKit
from leg_controller.legInfo import LB, LM, LF, RB, RM, RF, legReferencing

class hexMotorControl(Node):

    COXA_LEN = 71.5
    FEMUR_LEN = 100.02
    FOOT_HEIGHT = 0
    TIBIA_LEN = 150 + FOOT_HEIGHT
        
    leftLegsAddr: float
    rightLegsAddr: float

    #LB = legReferencing(False, 0, 1, 2, "Leg: <Left Back>")
    #LM = legReferencing(False, 3, 4, 5, "Leg: <Left Middle>")
    #LF = legReferencing(False, 6, 7, 8, "Leg: <Left Front>")
    #RB = legReferencing(True, 0, 1, 2, "Leg: <Right Back>")
    #RM = legReferencing(True, 3, 4, 5, "Leg: <Right Middle>")
    #RF = legReferencing(True, 6, 7, 8, "Leg: <Right Front>")

    def __init__(self):
        super().__init__("hex_legs_controller")
        # TODO: Add the addresses of the 2 i2c controllers
        self.kitL = ServoKit(channels=16, address=65) # 41 in hex
        self.kitR = ServoKit(channels=16, address=66) # 42 in hex
        
    def setLegServoAngles(self, legRef, hipAngle=float, shoulderAngle=float, kneeAngle=float):
        if legRef.side == False:
            self.kitL.servo[legRef.hip].angle = hipAngle
            self.kitL.servo[legRef.shoulder].angle = shoulderAngle
            self.kitL.servo[legRef.knee].angle = kneeAngle 

        elif legRef.side == True:
            self.kitR.servo[legRef.hip].angle = hipAngle
            self.kitR.servo[legRef.shoulder].angle = shoulderAngle
            self.kitR.servo[legRef.knee].angle = kneeAngle 

        self.get_logger().info(legRef.description + " moved to hip: " + str(hipAngle) + ", shoulder: " + str(hipAngle) + ", knee: " + str(kneeAngle))    


def main(args = None):
    rclpy.init(args = args)
    
    # Node
    node = hexMotorControl()  # create a node 
    
    node.setLegServoAngles(LB, 90, 90, 90)
    node.setLegServoAngles(RB, 90, 90, 90)
    sleep(2.0)

    node.setLegServoAngles(LB, 90 + 45, 90 + 45, 90 + 45)
    node.setLegServoAngles(RB, 90 + 45, 90 + 45, 90 + 45)
    sleep(2.0)

    #node.setLegServoAngles(LB, 90 - 45, 90 - 45, 90 - 45)
    #sleep(2.0)
    
    #rclpy.spin(node) # keep node alive 

    # End Node
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
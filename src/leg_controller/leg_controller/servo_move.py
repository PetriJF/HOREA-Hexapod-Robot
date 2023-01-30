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

    def __init__(self):
        super().__init__("hex_legs_controller")
        
        self.kitL = ServoKit(channels=16, address=65) # 41 in hex
        self.kitL.servo[0].set_pulse_width_range(500, 2500)
        
        self.kitR = ServoKit(channels=16, address=66) # 42 in hex
        self.kitR.servo[0].set_pulse_width_range(500, 2500)
        for i in range(0, 9):
            self.kitL.servo[i].set_pulse_width_range(500, 2500)
            self.kitR.servo[i].set_pulse_width_range(500, 2500)
            

    def setLegServoAngles(self, legRef, hipAngle=float, shoulderAngle=float, kneeAngle=float):
        if legRef.side == False:
            self.kitL.servo[legRef.hip].angle = hipAngle
            self.kitL.servo[legRef.shoulder].angle = shoulderAngle
            self.kitL.servo[legRef.knee].angle = kneeAngle 

        elif legRef.side == True:
            self.kitR.servo[legRef.hip].angle = hipAngle
            self.kitR.servo[legRef.shoulder].angle = shoulderAngle
            self.kitR.servo[legRef.knee].angle = kneeAngle 

        self.get_logger().info(legRef.description + " moved to hip: " + str(hipAngle) + ", shoulder: " + str(shoulderAngle) + ", knee: " + str(kneeAngle))    


def main(args = None):
    rclpy.init(args = args)
    
    # Node
    node = hexMotorControl()  # create a node 
    
    node.setLegServoAngles(LB, 30, 90, 90)
    node.setLegServoAngles(RB, 90, 90+30, 90+60)
    sleep(2.0)

    #node.setLegServoAngles(LB, 90 + 45, 90 + 45, 90 + 45)
    #node.setLegServoAngles(RB, 90 + 45, 90 + 45, 90 + 45)
    sleep(2.0)

    #node.setLegServoAngles(LB, 90 - 45, 90 - 45, 90 - 45)
    #sleep(2.0)
    
    rclpy.spin(node) # keep node alive 

    # End Node
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
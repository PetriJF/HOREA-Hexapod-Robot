#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import *
from adafruit_servokit import ServoKit
from leg_controller.legInfo import LB, LM, LF, RB, RM, RF, legReferencing
from hexapod_interfaces.msg import TargetAngles

class hexMotorControl(Node):    
    def __init__(self):
        super().__init__("hex_legs_controller")
        
        self.kitL = ServoKit(channels=16, address=65) # 41 in hex
        self.kitL.servo[0].set_pulse_width_range(500, 2500)
        
        self.kitR = ServoKit(channels=16, address=66) # 42 in hex
        self.kitR.servo[0].set_pulse_width_range(500, 2500)
        for i in range(0, 9):
            self.kitL.servo[i].set_pulse_width_range(500, 2500)
            self.kitR.servo[i].set_pulse_width_range(500, 2500)
        
        self.angleSubscriber = self.create_subscription(TargetAngles, "HexAngles", self.anglesCallback, 10)
            
    def anglesCallback(self, angleHex = TargetAngles):
        self.setLegServoAngles(RF, angleHex.shoulder_angle[1], angleHex.hip_angle[1], angleHex.knee_angle[1])
        self.setLegServoAngles(RM, angleHex.shoulder_angle[2], angleHex.hip_angle[2], angleHex.knee_angle[2])
        self.setLegServoAngles(RB, angleHex.shoulder_angle[3], angleHex.hip_angle[3], angleHex.knee_angle[3])
        self.setLegServoAngles(LB, angleHex.shoulder_angle[4], angleHex.hip_angle[4], angleHex.knee_angle[4])
        self.setLegServoAngles(LM, angleHex.shoulder_angle[5], angleHex.hip_angle[5], angleHex.knee_angle[5])
        self.setLegServoAngles(LF, angleHex.shoulder_angle[6], angleHex.hip_angle[6], angleHex.knee_angle[6])


    def setLegServoAngles(self, legRef=legReferencing, hipAngle=float, shoulderAngle=float, kneeAngle=float):
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
    
    #node.setLegServoAngles(LB, 30, 90, 90)
    #node.setLegServoAngles(RB, 90, 90+30, 90+60)
    #sleep(2.0)

    #node.setLegServoAngles(LB, 90 + 45, 90 + 45, 90 + 45)
    #node.setLegServoAngles(RB, 90 + 45, 90 + 45, 90 + 45)
    #sleep(2.0)

    #node.setLegServoAngles(LB, 90 - 45, 90 - 45, 90 - 45)
    #sleep(2.0)
    
    rclpy.spin(node) # keep node alive 

    # End Node
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
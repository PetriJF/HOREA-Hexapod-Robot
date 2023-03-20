#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from time import *
from adafruit_servokit import ServoKit
from hexapod_interfaces.msg import TargetAngles

class hexMotorControl(Node):    
    def __init__(self):
        super().__init__("hex_legs_controller")

        # Defining a descriptor for all the legs. Note 7 = PARAMETER_INTEGER_ARRAY!
        pd = ParameterDescriptor(description = "Port Definitions for each leg", type = 7) 

        # Declaring the parameters
        self.declare_parameter(name = "LB", descriptor = pd)
        self.declare_parameter(name = "LM", descriptor = pd)
        self.declare_parameter(name = "LF", descriptor = pd)
        self.declare_parameter(name = "RF", descriptor = pd)
        self.declare_parameter(name = "RM", descriptor = pd)
        self.declare_parameter(name = "RB", descriptor = pd)

        # Initializing the parameters
        self.portsRF = self.get_parameter("RF").value
        self.portsRM = self.get_parameter("RM").value
        self.portsRB = self.get_parameter("RB").value
        self.portsLF = self.get_parameter("LF").value
        self.portsLM = self.get_parameter("LM").value
        self.portsLB = self.get_parameter("LB").value

        self.get_logger().info("Parameters declared and initialized successfully!")

        # Defining the servo controllers
        self.kitL = ServoKit(channels=16, address=65) # 41 in hex
        self.kitR = ServoKit(channels=16, address=66) # 42 in hex
        
        # Setting the PWM ranges for all the servos
        for i in range(0, 16):
            self.kitL.servo[i].set_pulse_width_range(500, 2500)
            self.kitR.servo[i].set_pulse_width_range(500, 2500)
        
        self.get_logger().info("Servo controllers initialized and servos configured!")

        self.angleSubscriber = self.create_subscription(TargetAngles, "HexAngles", self.anglesCallback, 10)
        self.get_logger().info("Servo Controller Subscriber running")

    # Callback from the subsciber. Takes the angles fromt the TargetAngles array and moves the servos to the specific angle
    def anglesCallback(self, angleHex = TargetAngles):
        self.setLegServoAngles(True, self.portsRF, angleHex.shoulder_angle[1], angleHex.hip_angle[1], angleHex.knee_angle[1])
        self.setLegServoAngles(True, self.portsRM, angleHex.shoulder_angle[2], angleHex.hip_angle[2], angleHex.knee_angle[2])
        self.setLegServoAngles(True, self.portsRB, angleHex.shoulder_angle[3], angleHex.hip_angle[3], angleHex.knee_angle[3])
        self.setLegServoAngles(False, self.portsLB, angleHex.shoulder_angle[4], angleHex.hip_angle[4], angleHex.knee_angle[4])
        self.setLegServoAngles(False, self.portsLM, angleHex.shoulder_angle[5], angleHex.hip_angle[5], angleHex.knee_angle[5])
        self.setLegServoAngles(False, self.portsLF, angleHex.shoulder_angle[6], angleHex.hip_angle[6], angleHex.knee_angle[6])


    def setLegServoAngles(self, side = bool, legPorts = [int, int, int], hipAngle=float, shoulderAngle=float, kneeAngle=float):
        if side == False:
            self.kitL.servo[legPorts[0]].angle = hipAngle
            self.kitL.servo[legPorts[1]].angle = shoulderAngle
            self.kitL.servo[legPorts[2]].angle = kneeAngle 

        elif side == True:
            self.kitR.servo[legPorts[0]].angle = hipAngle
            self.kitR.servo[legPorts[1]].angle = shoulderAngle
            self.kitR.servo[legPorts[2]].angle = kneeAngle 

def main(args = None):
    rclpy.init(args = args)
    
    # Node
    node = hexMotorControl()  # create a node 
    
    rclpy.spin(node) # keep node alive 

    # End Node
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
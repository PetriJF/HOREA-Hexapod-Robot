#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, Float64, Float64MultiArray
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor
import time

import numpy as np

class TeleOp(Node):
    def __init__(self):
        super().__init__("teleop_node")

        pd = ParameterDescriptor(description = "parameter definition for the gait waypoint planning", type = 3) 
        self.declare_parameter(name = "base_width", descriptor = pd, value = 65.0)
        self.declare_parameter(name = "gait_width", descriptor = pd, value = 300.0)
        self.declare_parameter(name = "gait_altitude", descriptor = pd, value = 90.0)
        self.declare_parameter(name = "step_length", descriptor = pd, value = 50.0)
        self.declare_parameter(name = "base_altitude", descriptor= pd, value = 120.0)

        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value
        self.gait_altitude_ = self.get_parameter("gait_altitude").value
        self.step_length_ = self.get_parameter("step_length").value
        self.base_altitude_ = int(self.get_parameter("base_altitude").value)

        #self.action_server_ = ActionServer(self, StepManagement, "stepStatus", self.stepActionCallback)

        self.previous_animation_ = 1
        self.animation_ = Int64MultiArray()
        self.animation_.data = [ 1, 0 ]
        self.animation_command_list_ = ["1", "2", "3", "4"] 
        
        self.step_speed_ = Float64()
        self.step_speed_.data = 1.0
        self.prev_Button_ = -1
        self.gamepad_commands_ = self.create_subscription(Joy, 'joy', self.gamepadCallback, 10)
        
        self.animation_type_ = self.create_publisher(Int64MultiArray, 'animationType', 10)
        self.step_command_ = self.create_publisher(Float64MultiArray, 'stepCommands', 10)
        self.stepSpeedPub_ = self.create_publisher(Float64, 'step_speed', 10)
        self.baseInclinationPub_ = self.create_publisher(Float64MultiArray, 'base_inclination', 10)

    def gamepadCallback(self, cmd = Joy):
        step_command = Float64MultiArray()
        inclination_command = Float64MultiArray()

        # Left JoyStick controlling the crab walk
        crabMagnitude = np.maximum(np.abs(cmd.axes[0]), np.abs(cmd.axes[1]))
        if crabMagnitude != 0.0:
            crabAngle = (-1.0 * np.arctan2(cmd.axes[0], cmd.axes[1])) if cmd.axes[0] <= 0.0 else (2.0 * np.pi - np.arctan2(cmd.axes[0], cmd.axes[1]))
            self.get_logger().info("Going at " + str(crabAngle))
            step_command.data = [ crabAngle, self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]    
            self.step_command_.publish(step_command)
        
        # Right JoyStick for the base inclination modifier
        MIN_INCLINATION = np.deg2rad(-30.0)
        MAX_INCLINATION = np.deg2rad(30.0)
        inclinMag = np.maximum(np.abs(cmd.axes[2]), np.abs(cmd.axes[3]))
        if inclinMag != 0.0:
            xAxisInclination = MIN_INCLINATION + ((float(cmd.axes[2] - 1.0) / float(-1.0 - 1.0)) * (MAX_INCLINATION - MIN_INCLINATION))
            yAxisInclination = MIN_INCLINATION + ((float(cmd.axes[3] - 1.0) / float(-1.0 - 1.0)) * (MAX_INCLINATION - MIN_INCLINATION))
            
            inclination_command.data = [ xAxisInclination, yAxisInclination ]
            self.baseInclinationPub_.publish(inclination_command)
            time.sleep(0.5 * self.step_speed_.data * 0.01)

        # D-Pad left and right turning the robot in each direction
        if cmd.axes[6] != 0.0:
            turnAngle = cmd.axes[6] * ((np.pi / 2.0) + np.arcsin((self.step_length_) / (2.0 * self.gait_width_)))
            step_command.data = [ turnAngle, self.step_length_, self.gait_altitude_, self.gait_width_, 1.0 ]

            self.step_command_.publish(step_command)

        # D-Pad up and down representing the base height of the robot
        if (cmd.axes[7] < 0.0 and self.base_altitude_ > 60) or (cmd.axes[7] > 0.0 and self.base_altitude_ < 200.0):
            self.base_altitude_ = self.base_altitude_ + (cmd.axes[7] * 10)
            self.animation_.data = [ 10, int(self.base_altitude_) ]
            self.animation_type_.publish(self.animation_)

        # decongesting variable. Ensures that a button click is registered once
        decon = True if(cmd.buttons[self.prev_Button_] == 0) else False

        if decon:
            self.prev_Button_ = 0 # triangle not working because of it
            # Dealing with the animations using the L1 and R1 bumpers
            if cmd.buttons[6] == 1 and self.previous_animation_ == 2:
                self.previous_animation_ = 1
                self.get_logger().info("Getting in init pose")
                self.prev_Button_ = 6
                self.animation_.data = [ self.previous_animation_, 0 ]
                self.animation_type_.publish(self.animation_)
            if cmd.buttons[7] == 1 and self.previous_animation_ == 3:
                self.previous_animation_ = 4
                self.animation_.data = [ self.previous_animation_, 0 ]
                self.animation_type_.publish(self.animation_)
                self.prev_Button_ = 7
                self.previous_animation_ = 2
                self.get_logger().info("Lowering base")
            elif cmd.buttons[7] == 1 and self.previous_animation_ < 3:
                self.previous_animation_ = self.previous_animation_ + 1
                self.get_logger().info("Getting in pose " + str(self.previous_animation_))
                self.prev_Button_ = 7
                self.animation_.data = [ self.previous_animation_, 0 ]
                self.animation_type_.publish(self.animation_)
            
            # Dealing with the stepSpeed using L2 and R2 triggers
            if cmd.buttons[8] == 1 and self.step_speed_.data < 3.0:
                self.prev_Button_ = 8
                self.step_speed_.data = self.step_speed_.data + 0.25
                self.stepSpeedPub_.publish(self.step_speed_)
            if cmd.buttons[9] == 1 and self.step_speed_.data > 0.25:
                self.prev_Button_ = 9
                self.step_speed_.data = self.step_speed_.data - 0.25
                self.stepSpeedPub_.publish(self.step_speed_)
 

def main(args = None):
    rclpy.init(args = args)

    ctrl = TeleOp()
    try:
        rclpy.spin(ctrl)
    except KeyboardInterrupt:
        pass
    
    ctrl.get_logger().info("Shutting down..")
    ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
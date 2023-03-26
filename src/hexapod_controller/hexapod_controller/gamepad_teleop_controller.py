#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64MultiArray
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np

class TeleOp(Node):
    def __init__(self):
        super().__init__("teleop_node")

        pd = ParameterDescriptor(description = "parameter definition for the gait waypoint planning", type = 3) 
        self.declare_parameter(name = "base_width", descriptor = pd, value = 65.0)
        self.declare_parameter(name = "gait_width", descriptor = pd, value = 300.0)
        self.declare_parameter(name = "gait_altitude", descriptor = pd, value = 90.0)
        self.declare_parameter(name = "step_length", descriptor = pd, value = 50.0)

        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value
        self.gait_altitude_ = self.get_parameter("gait_altitude").value
        self.step_length_ = self.get_parameter("step_length").value

        #self.action_server_ = ActionServer(self, StepManagement, "stepStatus", self.stepActionCallback)

        self.previous_animation_ = Int64()
        self.previous_animation_.data = 1
        self.animation_command_list_ = ["1", "2", "3", "4"] 
        
        
        self.prev_Button_ = -1
        self.gamepad_commands_ = self.create_subscription(Joy, 'joy', self.gamepadCallback, 10)
        
        self.animation_type_ = self.create_publisher(Int64, 'animationType', 10)
        self.step_command_ = self.create_publisher(Float64MultiArray, 'stepCommands', 10)

    def gamepadCallback(self, cmd = Joy):
        command = Float64MultiArray()
        # Left JoyStick controlling the crab walk
        crabMagnitude = np.maximum(np.abs(cmd.axes[0]), np.abs(cmd.axes[1]))
        if crabMagnitude != 0.0:
            crabAngle = (-1.0 * np.arctan2(cmd.axes[0], cmd.axes[1])) if cmd.axes[0] <= 0.0 else (2.0 * np.pi - np.arctan2(cmd.axes[0], cmd.axes[1]))
            self.get_logger().info("Going at " + str(crabAngle))
            command.data = [ crabAngle, self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]    
            self.step_command_.publish(command)
        
        # D-Pad left and right turning the robot in each direction
        if cmd.axes[4] != 0.0:
            turnAngle = cmd.axes[4] * ((np.pi / 2.0) + np.arcsin((self.step_length_) / (2.0 * self.gait_width_)))
            command.data = [ turnAngle, self.step_length_, self.gait_altitude_, self.gait_width_, 1.0 ]

            self.get_logger().info("Rotating at " + str(turnAngle))

            self.step_command_.publish(command)

        decon = True if(cmd.buttons[self.prev_Button_] == 0) else False

        if decon:
            self.prev_Button_ = 0 # triangle not working because of it
            # Dealing with the animations using the L1 and R1 bumpers
            if cmd.buttons[4] == 1 and self.previous_animation_.data == 2:
                self.previous_animation_.data = 1
                self.get_logger().info("Getting in init pose")
                self.prev_Button_ = 4
                self.animation_type_.publish(self.previous_animation_)
            if cmd.buttons[5] == 1 and self.previous_animation_.data == 3:
                self.previous_animation_.data = 4
                self.animation_type_.publish(self.previous_animation_)
                self.prev_Button_ = 5
                self.previous_animation_.data = 2
                self.get_logger().info("Lowering base")
            elif cmd.buttons[5] == 1 and self.previous_animation_.data < 3:
                self.previous_animation_.data = self.previous_animation_.data + 1
                self.get_logger().info("Getting in pose " + str(self.previous_animation_))
                self.prev_Button_ = 5
                self.animation_type_.publish(self.previous_animation_)
 
            
        #self.get_logger().info(str(crabMagnitude) + " " + str(np.rad2deg(crabAngle)))
 

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
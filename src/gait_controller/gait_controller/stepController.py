#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np

## OLD NOT IN USE! KEPT FOR REFERENCE!!!

class Stepper(Node):
    def __init__(self):
        super().__init__("step_ctrl_node")
        self.get_logger().info("Init")
        
        pd = ParameterDescriptor(description = "parameter definition for the gait waypoint planning", type = 3) 
        self.declare_parameter(name = "base_width", descriptor = pd, value = 65.0)
        self.declare_parameter(name = "gait_width", descriptor = pd, value = 330.0)
        self.declare_parameter(name = "gait_altitude", descriptor = pd, value = 90.0)
        self.declare_parameter(name = "step_length", descriptor = pd, value = 50.0)

        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value
        self.gait_altitude_ = self.get_parameter("gait_altitude").value
        self.step_length_ = self.get_parameter("step_length").value

        self.posSub = self.create_subscription(String, 'stepType', self.stepCommand, 10)
        # Change to a float array
        self.step_command_ = self.create_publisher(Float64MultiArray, 'stepCommands', 10)

    def stepCommand(self, cmd = String):
        step_description = Float64MultiArray()

        if cmd.data == "w":
            step_description.data = [ np.deg2rad(0.0), self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]
            
            self.step_command_.publish(step_description)
        elif cmd.data == "a":
            step_description.data = [ np.deg2rad(270.0), self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]

            self.step_command_.publish(step_description)
        elif cmd.data == "s":
            step_description.data = [ np.deg2rad(180.0), self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]

            self.step_command_.publish(step_description)
        elif cmd.data == "d":
            step_description.data = [ np.deg2rad(90.0), self.step_length_, self.gait_altitude_, self.gait_width_, 0.0 ]
            
            self.step_command_.publish(step_description)
        elif cmd.data == "q":
            angle = (np.pi / 2.0) + np.arcsin((self.step_length_) / (2.0 * self.gait_width_))
            step_description.data = [ angle, self.step_length_, self.gait_altitude_, self.gait_width_, 1.0 ]

            self.step_command_.publish(step_description)
        elif cmd.data == "e":
            angle = -1.0 * ((np.pi / 2.0) + np.arcsin((self.step_length_) / (2.0 * self.gait_width_)))
            step_description.data = [ angle, self.step_length_, self.gait_altitude_, self.gait_width_, 1.0 ]

            self.step_command_.publish(step_description)
        else:
            self.get_logger().warning("Incorrect command sent to the step controller!!")
        
def main(args = None):
    rclpy.init(args = args)

    step = Stepper()
    try:
        rclpy.spin(step)
    except KeyboardInterrupt:
        pass
    
    step.get_logger().info("Shutting down..")
    step.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
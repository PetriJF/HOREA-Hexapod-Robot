#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray, Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np


class TeleOp(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.get_logger().info("Init")
        
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

        self.animation_command_list_ = ["1", "2", "3", "4"]
        self.step_command_list_ = ["w", "a", "s", "d", "q", "e"] 
        self.animation_type_ = self.create_publisher(Int64MultiArray, 'animationType', 10)
        self.step_command_ = self.create_publisher(Float64MultiArray, 'stepCommands', 10)

    
    def animCommandHandler(self, cmd = int):
        if str(cmd) in self.animation_command_list_:
            anim = Int64MultiArray()
            anim.data = [ cmd, 0 ]
            self.animation_type_.publish(anim)
        else:
            print("Wrong Animation Input! Please try again")
        
    def stepCommandHandler(self, cmd = String):
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

    ctrl = TeleOp()

    stepCmd = String()
  
    print("List of commands:\n\t1. Legs Up\n\t2. Legs Preped\n\t3. Raise Base\n\t4. Lower Base\n\tw - forward\n\ta - left\n\ts - backwards\n\td - right\n\tq - spin left\n\te - spin right\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    
    userInput = input("\nEnter command: ")

    while userInput != 'c':
        try:
            if userInput in ctrl.step_command_list_:
                stepCmd.data = str(userInput)
                ctrl.stepCommandHandler(stepCmd)
            elif userInput in ctrl.animation_command_list_:
                animCmd = int(userInput)
                ctrl.animCommandHandler(animCmd)
            else:
                print("Command not in the options! Try again")    
            userInput = input("\nEnter command: ")
        except KeyboardInterrupt:
            userInput = 'c'

    ctrl.get_logger().info("Shutting down..")
    ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
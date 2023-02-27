#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

## Bézier <3
class Stepper(Node):
    def __init__(self):
        super().__init__("step_ctrl_node")
        self.get_logger().info("Init")
        self.command_ = self.create_publisher(Int64, 'StepCommands', 10)

    def stepCommand(self, cmd = Int64):
        self.command_.publish(cmd)

        

def main(args = None):
    rclpy.init(args = args)

    step = Stepper()
    print("List of commands:\n\t1. Step Right Dominant\n\t2. Step Left Dominant\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    

    userInput = input("\nEnter command: ")
    cmd = Int64()

    while userInput != 'c':
        if (userInput == '1'):
            cmd.data = 1
            step.stepCommand(cmd)
            print("Step Right Dominant")
        elif (userInput == '2'):
            cmd.data = 2
            step.stepCommand(cmd)
            print("Step Left Dominant")
        else:
            print("Wrong command")
        
        userInput = input("\nEnter command: ")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
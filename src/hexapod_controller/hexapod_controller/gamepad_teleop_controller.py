#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
from sensor_msgs.msg import Joy
import numpy as np

class TeleOp(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.animation_command_list_ = [1, 2, 3, 4]
        self.step_command_list_ = ["w", "a", "s", "d", "q", "e"] 

        self.gamepad_commands_ = self.create_subscription(Joy, 'joy', self.gamepadCallback, 10)
        self.animation_type_ = self.create_publisher(Int64, 'animationType', 10)
        self.step_type_ = self.create_publisher(String, 'stepType', 10)

    def gamepadCallback(self, cmd = Joy):
        crabMagnitude = (np.abs(cmd.axes[0]) + np.abs(cmd.axes[1])) / 2.0
        if crabMagnitude != 0.0:
            crabAngle = (-1.0 * np.arctan2(cmd.axes[0], cmd.axes[1])) if cmd.axes[0] <= 0.0 else (2.0 * np.pi - np.arctan2(cmd.axes[0], cmd.axes[1]))
            self.get_logger().info(str(crabMagnitude) + " " + str(np.rad2deg(crabAngle)))

    #def animCommandHandler(self, cmd = Int64):
    #    if cmd.data in self.animation_command_list_:
    #        self.animation_type_.publish(cmd)
    #    else:
    #        print("Wrong Animation Input! Please try again")
        
    #def stepCommandHandler(self, cmd = String):
    #    if cmd.data in self.step_command_list_:
    #        self.step_type_.publish(cmd)
    #    else:
    #        print("Wrong Input! Please try again")
        

def main(args = None):
    rclpy.init(args = args)

    ctrl = TeleOp()

    rclpy.spin(ctrl)
    #stepCmd = String()
    # animCmd = Int64()


    # print("List of commands:\n\t1. Legs Up\n\t2. Legs Preped\n\t3. Raise Base\n\t4. Lower Base\n\tw - forward\n\ta - left\n\ts - backwards\n\td - right\n\tq - spin left\n\te - spin right\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    
    # userInput = input("\nEnter command: ")

    # while userInput != 'c':
    #     if userInput in ['q', 'w', 'e', 'a', 's', 'd']:
    #         stepCmd.data = str(userInput)
    #         ctrl.stepCommandHandler(stepCmd)
    #     elif userInput in ['1', '2', '3', '4']:
    #         animCmd.data = int(userInput)
    #         ctrl.animCommandHandler(animCmd)
    #     else:
    #         print("Command not in the options! Try again")    
    #     userInput = input("\nEnter command: ")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
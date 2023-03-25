#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64

class TeleOp(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.animation_command_list_ = [1, 2, 3, 4]
        self.step_command_list_ = ["w", "a", "s", "d", "q", "e"] 
        self.animation_type_ = self.create_publisher(Int64, 'animationType', 10)
        self.step_type_ = self.create_publisher(String, 'stepType', 10)

    
    def animCommandHandler(self, cmd = Int64):
        if cmd.data in self.animation_command_list_:
            self.animation_type_.publish(cmd)
        else:
            print("Wrong Animation Input! Please try again")
        
    def stepCommandHandler(self, cmd = String):
        if cmd.data in self.step_command_list_:
            self.step_type_.publish(cmd)
        else:
            print("Wrong Input! Please try again")
        

def main(args = None):
    rclpy.init(args = args)

    ctrl = TeleOp()

    stepCmd = String()
    animCmd = Int64()

    print("List of commands:\n\t1. Legs Up\n\t2. Legs Preped\n\t3. Raise Base\n\t4. Lower Base\n\tw - forward\n\ta - left\n\ts - backwards\n\td - right\n\tq - spin left\n\te - spin right\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    
    userInput = input("\nEnter command: ")

    while userInput != 'c':
        try:
            if userInput in ['q', 'w', 'e', 'a', 's', 'd']:
                stepCmd.data = str(userInput)
                ctrl.stepCommandHandler(stepCmd)
            elif userInput in ['1', '2', '3', '4']:
                animCmd.data = int(userInput)
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
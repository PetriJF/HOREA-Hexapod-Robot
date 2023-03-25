#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit

class singleMotorControlDemo(Node):
    def __init__(self):
        super().__init__("servo_controller")
        self.counter_ = True
        self.create_timer(2.0, self.timer_callback) 
        self.kit = ServoKit(channels=16)

        self.kit.servo[0].set_pulse_width_range(500, 2500)
        self.kit.servo[0].angle = 0

    def timer_callback(self):
        if(self.counter_):
            self.get_logger().info("Servo Moved to 180")
            self.servo_rotate(180.0)
        else:
            self.get_logger().info("Servo Moved to 0")
            self.servo_rotate(0.0)

        self.counter_ = not self.counter_
    
    def servo_rotate(self, angle):
        self.kit.servo[0].angle = angle


def main(args = None):
    rclpy.init(args = args)
    
    # Node

    node = singleMotorControlDemo()  # create a node 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.get_logger().info("Shutting down..")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
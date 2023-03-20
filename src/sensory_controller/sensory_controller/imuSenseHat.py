#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sense_hat import SenseHat

class IMUNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        sense = SenseHat()
        sense.set_rotation(r = 270)
        sense.show_message("Test", scroll_speed=0.05, text_colour=[0,0,255])

        self.get_logger().info("Init")
    
    #def gyroscopeRead(self):
    #    gyroRead = self.sense.get_orientation()
    #    pitch = round(gyroRead["pitch"], 2) # front to back
    #    roll = round(gyroRead["roll"], 2)   # side to side
    #    yaw = round(gyroRead["yaw"], 2)     # around center axis
    #
    #    self.get_logger().info("Orientation: " + pitch + " " + roll + " " + yaw + "\n")

def main(args = None):
    rclpy.init(args = args)
    
    imu = IMUNode()
    #while(True):
    #    imu.gyroscopeRead()
    
    rclpy.spin(imu)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
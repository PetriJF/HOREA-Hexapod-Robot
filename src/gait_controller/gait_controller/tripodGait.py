#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TripodGait(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.get_logger().info("Init")
    
    # Node dealing with planning the triangular motion for each leg in order to achieve the tripod pattern gait
    # Returns 3 trajectory points for the legs
    def tripodGaitWayPointer(self, relativeDirDeg = float, stepLength = float, gaitAltitude = float, gaitWidth = float):
        # Note! 0 deg represents forward
        gaitWaypoints = Point()[6]
        

    
def main(args = None):
    rclpy.init(args = args)
    
    tpG = TripodGait()

    rclpy.spin(tpG)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
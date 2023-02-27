#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int64
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import WaypointSetter

import numpy as np


class TripodGait(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.get_logger().info("Init")

        # Note 8 = PARAMETER_DOUBLE_ARRAY
        # Declaring the parameters representing each leg origin. Note 8 = PARAMETER_DOUBLE_ARRAY
        pd = ParameterDescriptor(description = "Origin definition for each leg", type = 8) 
        self.declare_parameter(name = "origin_RF", descriptor = pd)
        self.declare_parameter(name = "origin_RM", descriptor = pd)
        self.declare_parameter(name = "origin_RB", descriptor = pd)
        self.declare_parameter(name = "origin_LB", descriptor = pd)
        self.declare_parameter(name = "origin_LM", descriptor = pd)
        self.declare_parameter(name = "origin_LF", descriptor = pd)
        self.declare_parameter(name = "leg_angular_orientation", descriptor = pd)
        
        # Note 3 = PARAMETER_DOUBLE
        pd = ParameterDescriptor(description = "parameter definition for the gait waypoint planning", type = 3) 
        self.declare_parameter(name = "base_width", descriptor = pd, value = 65.0)
        self.declare_parameter(name = "gait_width", descriptor = pd, value = 300.0)
        self.declare_parameter(name = "gait_altitude", descriptor = pd, value = 90.0)
        self.declare_parameter(name = "step_length", descriptor = pd, value = 75.0)
        self.declare_parameter(name = "gait_speed", descriptor = pd, value = 1.0)
        
        #self.origins_ = [
        #    self.setPoint(self.get_parameter("origin_RF").value[0], self.get_parameter("origin_RF").value[1], 0.0),
        #    self.setPoint(self.get_parameter("origin_RM").value[0], self.get_parameter("origin_RM").value[1], 0.0),
        #    self.setPoint(self.get_parameter("origin_RB").value[0], self.get_parameter("origin_RB").value[1], 0.0),
        #    self.setPoint(self.get_parameter("origin_LB").value[0], self.get_parameter("origin_LB").value[1], 0.0),
        #    self.setPoint(self.get_parameter("origin_LM").value[0], self.get_parameter("origin_LM").value[1], 0.0),
        #    self.setPoint(self.get_parameter("origin_LF").value[0], self.get_parameter("origin_LF").value[1], 0.0)
        #]
        
        # Note gamma_ values : 0 RF, 1 RM, 2 RB, 3 LB, 4 LM, 5 LF 
        self.gamma_ = np.deg2rad(self.get_parameter("leg_angular_orientation").value)

        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value
        self.gait_altitude_ = self.get_parameter("gait_altitude").value
        self.step_length_ = self.get_parameter("step_length").value

        self.leg_waypoints_ = WaypointSetter()
        # Initialize the publisher to the tranjectory planner
        self.sub = self.create_subscription(Int64, 'StepCommands', self.commandsCallback, 10)
        self.wp_pub = self.create_publisher(WaypointSetter,'WaypointPlanner', 10)

    def commandsCallback(self, cmd = Int64):
        self.get_logger().info("Angles" +
                                str(self.gamma_[0]) + " " + 
                                str(self.gamma_[1]) + " " + 
                                str(self.gamma_[2]) + " " + 
                                str(self.gamma_[3]) + " " + 
                                str(self.gamma_[4]) + " " + 
                                str(self.gamma_[5]) 
                              )
        if cmd.data == 1:
            self.wayPointer(0.0, self.step_length_, self.gait_altitude_, self.gait_width_, True)
            self.get_logger().info("Step Command Received")
        elif cmd.data == 2:
            self.wayPointer(0.0, self.step_length_, self.gait_altitude_, self.gait_width_, False)
            self.get_logger().info("Step Command Received")

    def wayPointer(self, relativeDirRad = float, stepLength = float, gaitAltitude = float, gaitWidth = float, rightDominant = bool):
        
        self.leg_waypoints_.rf = self.bezierWaypointer4P(0, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.rm = self.bezierWaypointer4P(1, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.rb = self.bezierWaypointer4P(2, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.lb = self.bezierWaypointer4P(3, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.lm = self.bezierWaypointer4P(4, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.lf = self.bezierWaypointer4P(5, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
        self.leg_waypoints_.right_dominant = rightDominant

        # Publish to bezierTrajectory Node
        self.wp_pub.publish(self.leg_waypoints_)

    # Node dealing with planning the motion for each leg in order to achieve the tripod pattern gait
    # Returns 4 trajectory points for the legs, representing the 4 points needed for the bezier curve
    def bezierWaypointer4P(self, legIndex = int, relativeDirRad = float, stepLength = float, gaitAltitude = float, gaitWidth = float, rightDominant = bool):
        # Note! 0 deg represents forward
        A = Point()
        B = Point()
        C = Point()
        D = Point()

        B_width_ratio = 1/2
        B_height_ratio = 4/5

        C_width_ratio = 6/5
        C_height_ratio = 1

        #self.get_logger().info(str(self.origins_[legIndex].x) + " " + str(self.origins_[legIndex].y))

        A.x = gaitWidth * np.sin(self.gamma_[legIndex])
        A.y = gaitWidth * np.cos(self.gamma_[legIndex])
        A.z = 0.0

        # Differentiates between the first and second triangles in the tripod gait
        direction = 1 if (legIndex % 2 == 1) else -1
        if rightDominant == False:
            self.get_logger().info("Reversing")
            direction = direction * -1

        B.x = A.x + direction * (stepLength * np.sin(relativeDirRad)) * B_width_ratio 
        B.y = A.y + direction * (stepLength * np.cos(relativeDirRad)) * B_width_ratio
        B.z = B_height_ratio * (gaitAltitude)

        C.x = A.x + direction * (stepLength * np.sin(relativeDirRad)) * C_width_ratio
        C.y = A.y + direction * (stepLength * np.cos(relativeDirRad)) * C_width_ratio
        C.z = C_height_ratio * gaitAltitude

        D.x = A.x + direction * stepLength * np.sin(relativeDirRad) 
        D.y = A.y + direction * stepLength * np.cos(relativeDirRad) 
        D.z = 0.0

        self.get_logger().info("Index " + str(legIndex) 
                               + "\n\tA: " + str(A.x) + " " + str(A.y) + " " + str(A.z)
                               + "\n\tB: " + str(B.x) + " " + str(B.y) + " " + str(B.z)
                               + "\n\tC: " + str(C.x) + " " + str(C.y) + " " + str(C.z)
                               + "\n\tD: " + str(D.x) + " " + str(D.y) + " " + str(D.z)
                               )

        return [A, B, C, D]
            

    def setPoint(self, xT = float, yT = float, zT = float):
        tempPoint = Point()
        tempPoint.x = float(xT)
        tempPoint.y = float(yT)
        tempPoint.z = float(zT)
        
        return tempPoint

    
def main(args = None):
    rclpy.init(args = args)
    
    tpG = TripodGait()

    rclpy.spin(tpG)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
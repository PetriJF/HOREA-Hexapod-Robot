#!/usr/bin/env python3
import rclpy
from time import *
from rclpy.node import Node
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import WaypointSetter, TargetPositions

import numpy as np

## Bézier <3
class BezierTrajectory(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.get_logger().info("Init")

        pd = ParameterDescriptor(description = "Trajectory Parameters", type = 3) 
        self.declare_parameter(name = "resolution", descriptor = pd, value = 0.01)
        self.declare_parameter(name = "iter_delay", descriptor = pd, value = 0.001)

        self.resolution_ = self.get_parameter("resolution").value
        self.iter_delay_ = self.get_parameter("iter_delay").value

        self.targetPos_ = TargetPositions()
        self.targetPos_.x_pos[0] = 0.0
        self.targetPos_.y_pos[0] = 0.0
        self.targetPos_.z_pos[0] = 0.0

        self.posPub_ = self.create_publisher(TargetPositions, 'HexLegPos', 10)
        self.wpSub_ = self.create_subscription(WaypointSetter, 'WaypointPlanner', self.trajectoryPlannerCallBack, 10)

    def trajectoryPlannerCallBack(self, wp = WaypointSetter):
        # "time" segment representing the movement from the beginning point (0) to the end point(1)
        self.get_logger().info("Frame 1 started")

        # FIRST FRAME OF THE STEP
        for t in np.arange(0, 1 + self.resolution_, self.resolution_):
            if wp.right_dominant:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.bezier4P(wp.rm[0], wp.rm[1], wp.rm[2], wp.rm[3], t), 2)
                self.setTargPosIndex(self.bezier4P(wp.lb[0], wp.lb[1], wp.lb[2], wp.lb[3], t), 4)
                self.setTargPosIndex(self.bezier4P(wp.lf[0], wp.lf[1], wp.lf[2], wp.lf[3], t), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.linear2P(wp.rf[0], wp.rf[3], t), 1)
                self.setTargPosIndex(self.linear2P(wp.rb[0], wp.rb[3], t), 3)
                self.setTargPosIndex(self.linear2P(wp.lm[0], wp.lm[3], t), 5)
            else:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.linear2P(wp.rm[0], wp.rm[3], t), 2)
                self.setTargPosIndex(self.linear2P(wp.lb[0], wp.lb[3], t), 4)
                self.setTargPosIndex(self.linear2P(wp.lf[0], wp.lf[3], t), 6)
                
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.bezier4P(wp.rf[0], wp.rf[1], wp.rf[2], wp.rf[3], t), 1)
                self.setTargPosIndex(self.bezier4P(wp.rb[0], wp.rb[1], wp.rb[2], wp.rb[3], t), 3)
                self.setTargPosIndex(self.bezier4P(wp.lm[0], wp.lm[1], wp.lm[2], wp.lm[3], t), 5)

            self.get_logger().info("Target Pos:\n\t" + ' '.join(str(e) for e in self.targetPos_.x_pos) +
                                    "\n\t" + ' '.join(str(e) for e in self.targetPos_.y_pos) + 
                                    "\n\t" + ' '.join(str(e) for e in self.targetPos_.z_pos))
            self.posPub_.publish(self.targetPos_)
            
            ## Publish Current P
            sleep(self.iter_delay_)
       
        ## Final for

        self.get_logger().info("Frame 1 ended")

        # SECOND FRAME OF THE STEP
        for t in np.arange(0, 1 + self.resolution_, self.resolution_):
            if wp.right_dominant:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.linear2P(wp.rm[3], wp.rm[0], t), 2)
                self.setTargPosIndex(self.linear2P(wp.lb[3], wp.lb[0], t), 4)
                self.setTargPosIndex(self.linear2P(wp.lf[3], wp.lf[0], t), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.bezier4P(wp.rf[3], wp.rf[2], wp.rf[1], wp.rf[0], t), 1)
                self.setTargPosIndex(self.bezier4P(wp.rb[3], wp.rb[2], wp.rb[1], wp.rb[0], t), 3)
                self.setTargPosIndex(self.bezier4P(wp.lm[3], wp.lm[2], wp.lm[1], wp.lm[0], t), 5)
                
            else:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.bezier4P(wp.rm[3], wp.rm[2], wp.rm[1], wp.rm[0], t), 2)
                self.setTargPosIndex(self.bezier4P(wp.lb[3], wp.lb[2], wp.lb[1], wp.lb[0], t), 4)
                self.setTargPosIndex(self.bezier4P(wp.lf[3], wp.lf[2], wp.lf[1], wp.lf[0], t), 6)
                
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.linear2P(wp.rf[3], wp.rf[0], t), 1)
                self.setTargPosIndex(self.linear2P(wp.rb[3], wp.rb[0], t), 3)
                self.setTargPosIndex(self.linear2P(wp.lm[3], wp.lm[0], t), 5)

            self.get_logger().info("Target Pos:\n\t" + ' '.join(str(e) for e in self.targetPos_.x_pos) +
                                    "\n\t" + ' '.join(str(e) for e in self.targetPos_.y_pos) + 
                                    "\n\t" + ' '.join(str(e) for e in self.targetPos_.z_pos))
            self.posPub_.publish(self.targetPos_)
            
            ## Publish Current P
            sleep(self.iter_delay_)
       
        ## Final for

        self.get_logger().info("Frame 1 ended")

    ## Bezier Curve Trajectory function using 4 control points and a time segment t between [0, 1]
    def bezier4P(self, A = Point(), B = Point(), C = Point(), D = Point(), t = float):
        P = Point()
    
        P.x = (1.0-t) * (1.0-t) * (1.0-t) * A.x + 3.0 * (1.0-t) * (1.0-t) * t * B.x + 3.0 * (1.0-t) * t * t * C.x + t * t * t * D.x
        P.y = (1.0-t) * (1.0-t) * (1.0-t) * A.y + 3.0 * (1.0-t) * (1.0-t) * t * B.y + 3.0 * (1.0-t) * t * t * C.y + t * t * t * D.y
        P.z = (1.0-t) * (1.0-t) * (1.0-t) * A.z + 3.0 * (1.0-t) * (1.0-t) * t * B.z + 3.0 * (1.0-t) * t * t * C.z + t * t * t * D.z

        
        return P

    ## Simple Linear Trajectory function between two points on the time segment t between [0, 1]
    def linear2P(self, A = Point(), B = Point(), t = float):
        P = Point()
        P.x = A.x + t * (B.x - A.x)
        P.y = A.y + t * (B.y - A.y)
        P.z = A.z + t * (B.z - A.z)
        
        return P
    
    ## Stupid function used to set the TargetPositions values until I fix the interface to be a point array
    def setTargPosIndex(self, P = Point(), index = int):
        self.targetPos_.x_pos[index] = np.round(P.x, 2)
        self.targetPos_.y_pos[index] = np.round(P.y, 2)
        self.targetPos_.z_pos[index] = np.round(P.z, 2)

        #self.get_logger().info("Trajectory point " + str(P.x) + " " + str(P.y) + " " + str(P.z))


def main(args = None):
    rclpy.init(args = args)

    bzTraj = BezierTrajectory()
    
    rclpy.spin(bzTraj)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
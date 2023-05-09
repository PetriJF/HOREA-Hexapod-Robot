#!/usr/bin/env python3
import rclpy
import yaml
from time import *
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import TargetPositions
from hexapod_interfaces.action import StepAnimator

import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

## Bézier <3
class BezierTrajectory(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.get_logger().info("Init")

        pd = ParameterDescriptor(description = "Trajectory Parameters", type = 3) 
        self.declare_parameter(name = "resolution", descriptor = pd, value = 0.01)
        self.declare_parameter(name = "step_duration", descriptor = pd, value = 1.0)
        self.declare_parameter(name = "default_gait", value = 0)
        self.declare_parameter(name = "gait_width", descriptor = pd, value = 300.0)
        
        pd = ParameterDescriptor(description = "Origin definition for each leg", type = 8) 
        self.declare_parameter(name = "leg_angular_orientation", descriptor = pd)
        
        # Note gamma_ values : 0 RF, 1 RM, 2 RB, 3 LB, 4 LM, 5 LF 
        self.gamma_ = np.deg2rad(self.get_parameter("leg_angular_orientation").value)
        self.gait_width_ = self.get_parameter("gait_width").value
        self.resolution_ = self.get_parameter("resolution").value
        self.step_duration_ = self.get_parameter("step_duration").value
        self.current_gait_ = self.get_parameter("default_gait").value

        gaitConfigPath = os.path.join(get_package_share_directory("gait_controller"), "config", "gaitDescriptors.yaml")
        with open(gaitConfigPath, 'r') as f:
            gaits = yaml.safe_load(f)
        # Form the gait dictionary
        gait_library_ = []
        gaitIndex = 0
        for gait in gaits:
            gait_library_.append({
                'name': gait['name'],
                'tog': gait['tog'],
                'RF': gait['RF'],
                'RM': gait['RM'],
                'RB': gait['RB'],
                'LB': gait['LB'],
                'LM': gait['LM'],
                'LF': gait['LF']
            })

        print(gait_library_)

        self.action_server_ = ActionServer(self, StepAnimator, "stepStatus", self.trajectoryPlannerCallBack)

        self.targetPos_ = TargetPositions()
        self.posPub_ = self.create_publisher(TargetPositions, 'HexLegPos', 10)
        self.speedSub_ = self.create_subscription(Float64, 'step_speed', self.robotSpeedCallback, 10)
        self.gaitSub_ = self.create_subscription(Int64, 'current_gait', self.gaitSetterCallback, 10)


    def trajectoryPlannerCallBack(self, goal_handle):
        wp = goal_handle.request.waypointer
        feedback_msg = StepAnimator.Feedback()
        
       # for t in np.arange(0, 1 + self.resolution_, self.resolution_):


        # FIRST FRAME OF THE STEP
        for t in np.arange(0, 1 + self.resolution_, self.resolution_):
            feedback_msg.percentage = t / 2.0
            goal_handle.publish_feedback(feedback_msg)
            
            #self.setTargPosIndex(self.linear2P(wp.rf[0], wp.rf[3], t), 1)
            #self.setTargPosIndex(self.bezier4P(wp.rm[0], wp.rm[1], wp.rm[2], wp.rm[3], t), 2)
            #self.setTargPosIndex(self.linear2P(wp.rb[0], wp.rb[3], t), 3)
            #self.setTargPosIndex(self.bezier4P(wp.lb[0], wp.lb[1], wp.lb[2], wp.lb[3], t), 4)
            #self.setTargPosIndex(self.linear2P(wp.lm[0], wp.lm[3], t), 5)
            #self.setTargPosIndex(self.bezier4P(wp.lf[0], wp.lf[1], wp.lf[2], wp.lf[3], t), 6)

            if wp.right_dominant:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.bezier4P(wp.rm[0], wp.rm[1], wp.rm[2], wp.rm[3], t, 1), 2)
                self.setTargPosIndex(self.bezier4P(wp.lb[0], wp.lb[1], wp.lb[2], wp.lb[3], t, 3), 4)
                self.setTargPosIndex(self.bezier4P(wp.lf[0], wp.lf[1], wp.lf[2], wp.lf[3], t, 5), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.linear2P(wp.rf[0], wp.rf[3], t, 0), 1)
                self.setTargPosIndex(self.linear2P(wp.rb[0], wp.rb[3], t, 2), 3)
                self.setTargPosIndex(self.linear2P(wp.lm[0], wp.lm[3], t, 4), 5)
            else:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.linear2P(wp.rm[0], wp.rm[3], t, 1), 2)
                self.setTargPosIndex(self.linear2P(wp.lb[0], wp.lb[3], t, 3), 4)
                self.setTargPosIndex(self.linear2P(wp.lf[0], wp.lf[3], t, 5), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.bezier4P(wp.rf[0], wp.rf[1], wp.rf[2], wp.rf[3], t, 0), 1)
                self.setTargPosIndex(self.bezier4P(wp.rb[0], wp.rb[1], wp.rb[2], wp.rb[3], t, 2), 3)
                self.setTargPosIndex(self.bezier4P(wp.lm[0], wp.lm[1], wp.lm[2], wp.lm[3], t, 4), 5)

            self.posPub_.publish(self.targetPos_)

            sleep(0.5 * self.step_duration_ * self.resolution_)

        # SECOND FRAME OF THE STEP
        for t in np.arange(0, 1 + self.resolution_, self.resolution_):
            feedback_msg.percentage = 0.5 + t / 2.0
            goal_handle.publish_feedback(feedback_msg)

            if wp.right_dominant:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.linear2P(wp.rm[3], wp.rm[0], t, 1), 2)
                self.setTargPosIndex(self.linear2P(wp.lb[3], wp.lb[0], t, 3), 4)
                self.setTargPosIndex(self.linear2P(wp.lf[3], wp.lf[0], t, 5), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.bezier4P(wp.rf[3], wp.rf[2], wp.rf[1], wp.rf[0], t, 0), 1)
                self.setTargPosIndex(self.bezier4P(wp.rb[3], wp.rb[2], wp.rb[1], wp.rb[0], t, 2), 3)
                self.setTargPosIndex(self.bezier4P(wp.lm[3], wp.lm[2], wp.lm[1], wp.lm[0], t, 4), 5)
            else:
                # First Triangle (RM, LB, LF)
                self.setTargPosIndex(self.bezier4P(wp.rm[3], wp.rm[2], wp.rm[1], wp.rm[0], t, 1), 2)
                self.setTargPosIndex(self.bezier4P(wp.lb[3], wp.lb[2], wp.lb[1], wp.lb[0], t, 3), 4)
                self.setTargPosIndex(self.bezier4P(wp.lf[3], wp.lf[2], wp.lf[1], wp.lf[0], t, 5), 6)
                # Second Triangle(RF, RB, LM)
                self.setTargPosIndex(self.linear2P(wp.rf[3], wp.rf[0], t, 0), 1)
                self.setTargPosIndex(self.linear2P(wp.rb[3], wp.rb[0], t, 2), 3)
                self.setTargPosIndex(self.linear2P(wp.lm[3], wp.lm[0], t, 4), 5)

            self.posPub_.publish(self.targetPos_)
            
            sleep(0.5 * self.step_duration_ * self.resolution_)

        goal_handle.succeed()
        
        result = StepAnimator.Result()
        result.completed_percentage = 1.0

        return result

    # def trajectoryPlannerCallBack(self, goal_handle):
    #     wp = goal_handle.request.waypointer
    #     feedback_msg = StepAnimator.Feedback()
        
    #     # FIRST FRAME OF THE STEP
    #     for t in np.arange(0, 1 + self.resolution_, self.resolution_):
    #         feedback_msg.percentage = t / 2.0
    #         goal_handle.publish_feedback(feedback_msg)
            
    #         if wp.right_dominant:
    #             # First Triangle (RM, LB, LF)
    #             self.setTargPosIndex(self.bezier4P(wp.rm[0], wp.rm[1], wp.rm[2], wp.rm[3], t), 2)
    #             self.setTargPosIndex(self.bezier4P(wp.lb[0], wp.lb[1], wp.lb[2], wp.lb[3], t), 4)
    #             self.setTargPosIndex(self.bezier4P(wp.lf[0], wp.lf[1], wp.lf[2], wp.lf[3], t), 6)
    #             # Second Triangle(RF, RB, LM)
    #             self.setTargPosIndex(self.linear2P(wp.rf[0], wp.rf[3], t), 1)
    #             self.setTargPosIndex(self.linear2P(wp.rb[0], wp.rb[3], t), 3)
    #             self.setTargPosIndex(self.linear2P(wp.lm[0], wp.lm[3], t), 5)
    #         else:
    #             # First Triangle (RM, LB, LF)
    #             self.setTargPosIndex(self.linear2P(wp.rm[0], wp.rm[3], t), 2)
    #             self.setTargPosIndex(self.linear2P(wp.lb[0], wp.lb[3], t), 4)
    #             self.setTargPosIndex(self.linear2P(wp.lf[0], wp.lf[3], t), 6)
    #             # Second Triangle(RF, RB, LM)
    #             self.setTargPosIndex(self.bezier4P(wp.rf[0], wp.rf[1], wp.rf[2], wp.rf[3], t), 1)
    #             self.setTargPosIndex(self.bezier4P(wp.rb[0], wp.rb[1], wp.rb[2], wp.rb[3], t), 3)
    #             self.setTargPosIndex(self.bezier4P(wp.lm[0], wp.lm[1], wp.lm[2], wp.lm[3], t), 5)

    #         self.posPub_.publish(self.targetPos_)

    #         sleep(0.5 * self.step_duration_ * self.resolution_)

    #     # SECOND FRAME OF THE STEP
    #     for t in np.arange(0, 1 + self.resolution_, self.resolution_):
    #         feedback_msg.percentage = 0.5 + t / 2.0
    #         goal_handle.publish_feedback(feedback_msg)

    #         if wp.right_dominant:
    #             # First Triangle (RM, LB, LF)
    #             self.setTargPosIndex(self.linear2P(wp.rm[3], wp.rm[0], t), 2)
    #             self.setTargPosIndex(self.linear2P(wp.lb[3], wp.lb[0], t), 4)
    #             self.setTargPosIndex(self.linear2P(wp.lf[3], wp.lf[0], t), 6)
    #             # Second Triangle(RF, RB, LM)
    #             self.setTargPosIndex(self.bezier4P(wp.rf[3], wp.rf[2], wp.rf[1], wp.rf[0], t), 1)
    #             self.setTargPosIndex(self.bezier4P(wp.rb[3], wp.rb[2], wp.rb[1], wp.rb[0], t), 3)
    #             self.setTargPosIndex(self.bezier4P(wp.lm[3], wp.lm[2], wp.lm[1], wp.lm[0], t), 5)
    #         else:
    #             # First Triangle (RM, LB, LF)
    #             self.setTargPosIndex(self.bezier4P(wp.rm[3], wp.rm[2], wp.rm[1], wp.rm[0], t), 2)
    #             self.setTargPosIndex(self.bezier4P(wp.lb[3], wp.lb[2], wp.lb[1], wp.lb[0], t), 4)
    #             self.setTargPosIndex(self.bezier4P(wp.lf[3], wp.lf[2], wp.lf[1], wp.lf[0], t), 6)
    #             # Second Triangle(RF, RB, LM)
    #             self.setTargPosIndex(self.linear2P(wp.rf[3], wp.rf[0], t), 1)
    #             self.setTargPosIndex(self.linear2P(wp.rb[3], wp.rb[0], t), 3)
    #             self.setTargPosIndex(self.linear2P(wp.lm[3], wp.lm[0], t), 5)

    #         self.posPub_.publish(self.targetPos_)
            
    #         sleep(0.5 * self.step_duration_ * self.resolution_)

    #     goal_handle.succeed()
        
    #     result = StepAnimator.Result()
    #     result.completed_percentage = 1.0

    #     return result

    ## Bezier Curve Trajectory function using 4 control points and a time segment t between [0, 1]
    def bezier4P(self, A = Point(), B = Point(), C = Point(), D = Point(), t = float, index = int):
        P = Point()

        P.x = (1.0-t) * (1.0-t) * (1.0-t) * (A.x + self.gait_width_ * np.cos(self.gamma_[index])) + 3.0 * (1.0-t) * (1.0-t) * t * (B.x + self.gait_width_ * np.cos(self.gamma_[index])) + 3.0 * (1.0-t) * t * t * (C.x + self.gait_width_ * np.cos(self.gamma_[index])) + t * t * t * (D.x + self.gait_width_ * np.cos(self.gamma_[index]))
        P.y = (1.0-t) * (1.0-t) * (1.0-t) * (A.y + self.gait_width_ * np.sin(self.gamma_[index])) + 3.0 * (1.0-t) * (1.0-t) * t * (B.y + self.gait_width_ * np.sin(self.gamma_[index])) + 3.0 * (1.0-t) * t * t * (C.y + self.gait_width_ * np.sin(self.gamma_[index])) + t * t * t * (D.y + self.gait_width_ * np.sin(self.gamma_[index]))
        P.z = (1.0-t) * (1.0-t) * (1.0-t) * A.z + 3.0 * (1.0-t) * (1.0-t) * t * B.z + 3.0 * (1.0-t) * t * t * C.z + t * t * t * D.z

        return P

    ## Simple Linear Trajectory function between two points on the time segment t between [0, 1]
    def linear2P(self, A = Point(), B = Point(), t = float, index = int):
        P = Point()

        P.x = A.x + self.gait_width_ * np.cos(self.gamma_[index]) + t * (B.x - A.x)
        P.y = A.y + self.gait_width_ * np.sin(self.gamma_[index]) + t * (B.y - A.y)
        P.z = A.z + t * (B.z - A.z)
        
        return P
    
    def robotSpeedCallback(self, speed = Float64):
        self.step_duration_ = speed.data

    def gaitSetterCallback(self, gait = Int64):
        pass

    ## Stupid function used to set the TargetPositions values until I fix the interface to be a point array
    def setTargPosIndex(self, P = Point(), index = int):
        self.targetPos_.x_pos[index] = np.round(P.x, 2)
        self.targetPos_.y_pos[index] = np.round(P.y, 2)
        self.targetPos_.z_pos[index] = np.round(P.z, 2)

def main(args = None):
    rclpy.init(args = args)

    bzTraj = BezierTrajectory()
    
    try:
        rclpy.spin(bzTraj)
    except KeyboardInterrupt:
        pass
    
    bzTraj.get_logger().info("Shutting down..")
    bzTraj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
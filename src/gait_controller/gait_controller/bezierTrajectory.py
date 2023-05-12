#!/usr/bin/env python3
import rclpy
from time import *
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import TargetPositions
from hexapod_interfaces.action import StepAnimator


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
        self.declare_parameter(name = "step_length", descriptor = pd, value = 50.0)
        
        pd = ParameterDescriptor(description = "Origin definition for each leg", type = 8) 
        
        self.declare_parameter(name = "leg_angular_orientation", descriptor = pd)
        
        # Note gamma_ values : 0 RF, 1 RM, 2 RB, 3 LB, 4 LM, 5 LF 
        self.gamma_ = np.deg2rad(self.get_parameter("leg_angular_orientation").value)
        self.gait_width_ = self.get_parameter("gait_width").value
        self.resolution_ = self.get_parameter("resolution").value
        self.step_duration_ = self.get_parameter("step_duration").value
        self.current_gait_ = self.get_parameter("default_gait").value
        self.step_length_ = self.get_parameter("step_length").value

        self.action_server_ = ActionServer(self, StepAnimator, "stepStatus", self.trajectoryPlannerCallBack)

        self.targetPos_ = TargetPositions()
        self.posPub_ = self.create_publisher(TargetPositions, 'HexLegPos', 10)
        self.speedSub_ = self.create_subscription(Float64, 'step_speed', self.robotSpeedCallback, 10)

    def trajectoryPlannerCallBack(self, goal_handle):
        wp = goal_handle.request.waypointer
        feedback_msg = StepAnimator.Feedback()
        
        for t in np.arange(0, 1 + self.resolution_, self.resolution_):
            feedback_msg.percentage = t
            goal_handle.publish_feedback(feedback_msg)

            # RIGHT FRONT LEG
            if (t >= wp.rf_timings[0] and t < wp.rf_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.rf[2], wp.rf[3], wp.rf[4], wp.rf[5], t, wp.rf_timings[0], wp.rf_timings[1]), 1)
            elif t < wp.rf_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.rf[0], wp.rf[1], t, 0.0, wp.rf_timings[0]), 1)
            else:
                self.setTargPosIndex(self.linear2P(wp.rf[6], wp.rf[7], t, wp.rf_timings[1], 1.0), 1)
            # RIGHT MIDDLE LEG
            if (t >= wp.rm_timings[0] and t < wp.rm_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.rm[2], wp.rm[3], wp.rm[4], wp.rm[5], t, wp.rm_timings[0], wp.rm_timings[1]), 2)
            elif t < wp.rm_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.rm[0], wp.rm[1], t, 0.0, wp.rm_timings[0]), 2)
            else:
                self.setTargPosIndex(self.linear2P(wp.rm[6], wp.rm[7], t, wp.rm_timings[1], 1.0), 2)
            # RIGHT BACK LEG
            if (t >= wp.rb_timings[0] and t < wp.rb_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.rb[2], wp.rb[3], wp.rb[4], wp.rb[5], t, wp.rb_timings[0], wp.rb_timings[1]), 3)
            elif t < wp.rb_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.rb[0], wp.rb[1], t, 0.0, wp.rb_timings[0]), 3)
            else:
                self.setTargPosIndex(self.linear2P(wp.rb[6], wp.rb[7], t, wp.rb_timings[1], 1.0), 3)
            # LEFT BACK LEG
            if (t >= wp.lb_timings[0] and t < wp.lb_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.lb[2], wp.lb[3], wp.lb[4], wp.lb[5], t, wp.lb_timings[0], wp.lb_timings[1]), 4)
            elif t < wp.lb_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.lb[0], wp.lb[1], t, 0.0, wp.lb_timings[0]), 4)
            else:
                self.setTargPosIndex(self.linear2P(wp.lb[6], wp.lb[7], t, wp.lb_timings[1], 1.0), 4)
            # LEFT MIDDLE LEG
            if (t >= wp.lm_timings[0] and t < wp.lm_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.lm[2], wp.lm[3], wp.lm[4], wp.lm[5], t, wp.lm_timings[0], wp.lm_timings[1]), 5)
            elif t < wp.lm_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.lm[0], wp.lm[1], t, 0.0, wp.lm_timings[0]), 5)
            else:
                self.setTargPosIndex(self.linear2P(wp.lm[6], wp.lm[7], t, wp.lm_timings[1], 1.0), 5)
            # LEFT FRONT LEG
            if (t >= wp.lf_timings[0] and t < wp.lf_timings[1]):
                self.setTargPosIndex(self.bezier4P(wp.lf[2], wp.lf[3], wp.lf[4], wp.lf[5], t, wp.lf_timings[0], wp.lf_timings[1]), 6)
            elif t < wp.lf_timings[0]:
                self.setTargPosIndex(self.linear2P(wp.lf[0], wp.lf[1], t, 0.0, wp.lf_timings[0]), 6)
            else:
                self.setTargPosIndex(self.linear2P(wp.lf[6], wp.lf[7], t, wp.lf_timings[1], 1.0), 6)

            self.posPub_.publish(self.targetPos_)
            
            sleep(self.step_duration_ * self.resolution_)
            
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
    def bezier4P(self, A = Point(), B = Point(), C = Point(), D = Point(), t_raw = float, t_min = float, t_max = float):
        P = Point()
        
        t = ((t_raw - t_min) / (t_max - t_min)) if t_min != t_max else t_min

        P.x = (1.0-t) * (1.0-t) * (1.0-t) * A.x + 3.0 * (1.0-t) * (1.0-t) * t * B.x + 3.0 * (1.0-t) * t * t * C.x + t * t * t * D.x
        P.y = (1.0-t) * (1.0-t) * (1.0-t) * A.y + 3.0 * (1.0-t) * (1.0-t) * t * B.y + 3.0 * (1.0-t) * t * t * C.y + t * t * t * D.y
        P.z = (1.0-t) * (1.0-t) * (1.0-t) * A.z + 3.0 * (1.0-t) * (1.0-t) * t * B.z + 3.0 * (1.0-t) * t * t * C.z + t * t * t * D.z

        return P

    ## Simple Linear Trajectory function between two points on the time segment t between [0, 1]
    def linear2P(self, A = Point(), B = Point(), t_raw = float, t_min = float, t_max = float):
        P = Point()
        
        t = ((t_raw - t_min) / (t_max - t_min)) if t_min != t_max else t_min

        P.x = A.x + t * (B.x - A.x)
        P.y = A.y + t * (B.y - A.y)
        P.z = A.z + t * (B.z - A.z)
        
        return P
    
    def robotSpeedCallback(self, speed = Float64):
        self.step_duration_ = speed.data

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
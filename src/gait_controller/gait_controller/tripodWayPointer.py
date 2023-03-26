#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import WaypointSetter
from hexapod_interfaces.action import StepAnimator

import numpy as np


class TripodGait(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.get_logger().info("Init")

        # Note 8 = PARAMETER_DOUBLE_ARRAY
        # Declaring the parameters representing each leg origin. Note 8 = PARAMETER_DOUBLE_ARRAY
        pd = ParameterDescriptor(description = "Origin definition for each leg", type = 8) 
        self.declare_parameter(name = "leg_angular_orientation", descriptor = pd)
        
        # Note gamma_ values : 0 RF, 1 RM, 2 RB, 3 LB, 4 LM, 5 LF 
        self.gamma_ = np.deg2rad(self.get_parameter("leg_angular_orientation").value)
        # Represents if the right legs should step towards the desired direction first (True) or the left legs (False)
        self.last_step_type_ = True
        # Used for storing and transmitting the waypoints' information
        self.leg_waypoints_ = WaypointSetter()
        
        # Action Client and feedback for ensuring the step is done before trying to start another one
        self.action_client_ = ActionClient(self, StepAnimator, "stepStatus")  # Note this should be changes to have some sort of end location feedback
        self.feedback_ = 1.0

        # Initialize the publisher to the tranjectory planner
        self.sub = self.create_subscription(Float64MultiArray, 'stepCommands', self.commandsCallback, 10)

    def commandsCallback(self, cmd = Float64MultiArray):
        # Setting the information needed by the waypointer from the topic float array
        if self.feedback_ == 1.0:
            self.wayPointer(relativeDirRad = cmd.data[0],
                            stepLength = cmd.data[1], 
                            gaitAltitude = cmd.data[2],
                            gaitWidth = cmd.data[3], 
                            rightDominant = self.last_step_type_,
                            spin = False if (cmd.data[4] == 0.0) else True)
            # Change the next step to start with the other leg to the previous one. Makes the walk "animation look nicer"
            self.last_step_type_ = not self.last_step_type_


    def wayPointer(self, relativeDirRad = float, stepLength = float, gaitAltitude = float, gaitWidth = float, rightDominant = bool, spin = bool):
        if (not spin):
            # When the robot is not spinning in spot, all legs must aim for the robot relative angle in order to produce the movement
            self.leg_waypoints_.rf = self.bezierWaypointer4P(0, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rm = self.bezierWaypointer4P(1, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rb = self.bezierWaypointer4P(2, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lb = self.bezierWaypointer4P(3, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lm = self.bezierWaypointer4P(4, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lf = self.bezierWaypointer4P(5, relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.right_dominant = rightDominant
            
            goal_msg = StepAnimator.Goal()
            goal_msg.waypointer = self.leg_waypoints_
            self.action_client_.wait_for_server()

            self.send_goal_ = self.action_client_.send_goal_async(
                goal_msg,
                feedback_callback = self.feedbackCallback
            )

            self.send_goal_.add_done_callback(self.goal_response_callback)
        else:
            # When the robot is spinning in spot, all legs must move in terms of their own orientation 
            self.leg_waypoints_.rf = self.bezierWaypointer4P(0, self.gamma_[0] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rm = self.bezierWaypointer4P(1, self.gamma_[1] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rb = self.bezierWaypointer4P(2, self.gamma_[2] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lb = self.bezierWaypointer4P(3, self.gamma_[3] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lm = self.bezierWaypointer4P(4, self.gamma_[4] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lf = self.bezierWaypointer4P(5, self.gamma_[5] - relativeDirRad, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.right_dominant = rightDominant
            
            goal_msg = StepAnimator.Goal()
            goal_msg.waypointer = self.leg_waypoints_
            self.action_client_.wait_for_server()

            self.send_goal_ = self.action_client_.send_goal_async(
                goal_msg,
                feedback_callback = self.feedbackCallback
            )

            self.send_goal_.add_done_callback(self.goal_response_callback)
    
    # Represents the request status and rather it was accepted or not and how it the code continues forward
    def goal_response_callback(self, status):
        goal_handle = status.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Command rejected")
            return
        self.get_result_ = goal_handle.get_result_async()
        self.get_result_.add_done_callback(self.get_result_callback)

    # Getting the 100% done value representing that the step was done
    def get_result_callback(self, status):
        self.feedback_ = status.result().result.completed_percentage

    # Getting the current percentage feedback for the 
    def feedbackCallback(self, feedback_msg):
        self.feedback_ = feedback_msg.feedback.percentage

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


        A.x = gaitWidth * np.cos(self.gamma_[legIndex])
        A.y = gaitWidth * np.sin(self.gamma_[legIndex])
        A.z = 0.0

        # Differentiates between the first and second triangles in the tripod gait
        direction = 1 if (legIndex % 2 == 1) else -1
        if rightDominant == False:
            direction = direction * -1

        B.x = A.x + direction * (stepLength * np.cos(relativeDirRad)) * B_width_ratio 
        B.y = A.y + direction * (stepLength * np.sin(relativeDirRad)) * B_width_ratio
        B.z = B_height_ratio * (gaitAltitude)

        C.x = A.x + direction * (stepLength * np.cos(relativeDirRad)) * C_width_ratio
        C.y = A.y + direction * (stepLength * np.sin(relativeDirRad)) * C_width_ratio
        C.z = C_height_ratio * gaitAltitude

        D.x = A.x + direction * stepLength * np.cos(relativeDirRad) 
        D.y = A.y + direction * stepLength * np.sin(relativeDirRad) 
        D.z = 0.0

        #self.get_logger().info("Index " + str(legIndex) 
        #                       + "\n\tA: " + str(A.x) + " " + str(A.y) + " " + str(A.z)
        #                       + "\n\tB: " + str(B.x) + " " + str(B.y) + " " + str(B.z)
        #                       + "\n\tC: " + str(C.x) + " " + str(C.y) + " " + str(C.z)
        #                       + "\n\tD: " + str(D.x) + " " + str(D.y) + " " + str(D.z)
        #                       )

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

    try:
        rclpy.spin(tpG)
    except KeyboardInterrupt:
        pass
    
    tpG.get_logger().info("Shutting down..")
    tpG.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
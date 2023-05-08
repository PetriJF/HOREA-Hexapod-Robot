#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
from hexapod_interfaces.msg import WaypointSetter, StepDescriptor
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
        self.sub = self.create_subscription(StepDescriptor, 'stepCommands', self.commandsCallback, 10)

    def commandsCallback(self, cmd = StepDescriptor):
        # Setting the information needed by the waypointer from the topic float array
        if self.feedback_ == 1.0:
            self.wayPointer(direction = cmd.direction,
                            angle = cmd.angle,
                            stepLength = cmd.step_len, 
                            gaitAltitude = cmd.gait_alt,
                            gaitWidth = cmd.gait_wid, 
                            rightDominant = self.last_step_type_,
                            angularRepresentation = cmd.ang_rep,
                            dirComponent = cmd.dir_component,
                            angComponent = cmd.ang_component
            )
            # Change the next step to start with the other leg to the previous one. Makes the walk "animation look nicer"
            self.last_step_type_ = not self.last_step_type_


    def wayPointer(self, direction = float, angle = float, angularRepresentation = [float, float],
                   stepLength = float, gaitAltitude = float, gaitWidth = float, 
                   rightDominant = bool, dirComponent = bool, angComponent = bool):
        if dirComponent and not angComponent:
            self.get_logger().info("Direction")
            # When the robot is not spinning in spot, all legs must aim for the robot relative angle in order to produce the movement
            self.leg_waypoints_.rf = self.bezierWaypointer4P(0, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rm = self.bezierWaypointer4P(1, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rb = self.bezierWaypointer4P(2, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lb = self.bezierWaypointer4P(3, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lm = self.bezierWaypointer4P(4, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lf = self.bezierWaypointer4P(5, direction, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.right_dominant = rightDominant
            
            goal_msg = StepAnimator.Goal()
            goal_msg.waypointer = self.leg_waypoints_
            self.action_client_.wait_for_server()

            self.send_goal_ = self.action_client_.send_goal_async(
                goal_msg,
                feedback_callback = self.feedbackCallback
            )

            self.send_goal_.add_done_callback(self.goal_response_callback)
        elif angComponent and not dirComponent:
            self.get_logger().info("Spin")
            # When the robot is spinning in spot, all legs must move in terms of their own orientation 
            self.leg_waypoints_.rf = self.bezierWaypointer4P(0, self.gamma_[0] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rm = self.bezierWaypointer4P(1, self.gamma_[1] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.rb = self.bezierWaypointer4P(2, self.gamma_[2] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lb = self.bezierWaypointer4P(3, self.gamma_[3] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lm = self.bezierWaypointer4P(4, self.gamma_[4] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.lf = self.bezierWaypointer4P(5, self.gamma_[5] - angle, stepLength, gaitAltitude, gaitWidth, rightDominant)
            self.leg_waypoints_.right_dominant = rightDominant
            
            goal_msg = StepAnimator.Goal()
            goal_msg.waypointer = self.leg_waypoints_
            self.action_client_.wait_for_server()

            self.send_goal_ = self.action_client_.send_goal_async(
                goal_msg,
                feedback_callback = self.feedbackCallback
            )

            self.send_goal_.add_done_callback(self.goal_response_callback)
        elif angComponent and dirComponent:
            self.get_logger().info("Curve")
            
            # Getting the turning radius to form the turning circle
            r = gaitWidth + 10.0 + gaitWidth * np.maximum(np.abs(angularRepresentation[0]), np.abs(angularRepresentation[1])) 

            # Getting the 4 radiuses used to represent the 4 arcs of the legs
            r1 = r - gaitWidth
            r2 = r - gaitWidth * np.cos(np.pi / 3.0)
            r3 = r + gaitWidth * np.cos(np.pi / 3.0)
            r4 = r + gaitWidth

            # Getting the magnitudes for the each leg group
            tetha = 2.0 * np.arcsin(stepLength / (2.0 * r)) # Turning angle
            m1 = 2.0 * np.cos((np.pi - tetha) / 2.0) * r1   # Magnitude for the leg closest to the turning circle center M
            m2 = 2.0 * np.cos((np.pi - tetha) / 2.0) * r2   # Magnitude for legs closer to the turning circle from the circle center F, B
            m3 = 2.0 * np.cos((np.pi - tetha) / 2.0) * r3   # Magnitude for of the legs after the turning circle F, B
            m4 = 2.0 * np.cos((np.pi - tetha) / 2.0) * r4   # Magnitude for the futhest leg from the turning circle center M

            # tethaIn represents the reference angle for the legs closest to the turning circle's center, while
            # tethaOut represents the ones further.
            tethaIn = np.arccos((r*r + r2*r2 - gaitWidth*gaitWidth)/(2*r*r2))
            tethaOut = np.arccos((r*r + r3*r3 - gaitWidth*gaitWidth)/(2*r*r3))
            
            # Used to represent the two sets of angles for the front and back legs.
            INWARD = 0
            OUTWARD = 1

            pullAngle = [ (2 * tethaIn + tetha) / 2.0, (2 * tethaOut + tetha) / 2.0 ]
            centAngle = tetha / 2.0
            pushAngle = [ (-tetha - 2 * tethaIn) / 2.0, (-tetha - 2 * tethaOut) / 2.0 ]

            # EXPERIMENTAL
            # if (angularRepresentation[0] > 0.0 and angularRepresentation[1] > 0.0):
            #     self.get_logger().info("Forward left")
            #     self.leg_waypoints_.rf = self.bezierWaypointer4P(0, 2.0 * np.pi - pullAngle[OUTWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rm = self.bezierWaypointer4P(1, 2.0 * np.pi - centAngle, m1, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rb = self.bezierWaypointer4P(2, pushAngle[OUTWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lb = self.bezierWaypointer4P(3, pushAngle[INWARD], m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lm = self.bezierWaypointer4P(4, 2.0 * np.pi - centAngle, m4, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lf = self.bezierWaypointer4P(5, 2.0 * np.pi - pullAngle[INWARD] / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.right_dominant = rightDominant
            # elif (angularRepresentation[0] < 0.0 and angularRepresentation[1] > 0.0):
            #     self.get_logger().info("Forward right")
            #     self.leg_waypoints_.rf = self.bezierWaypointer4P(0, pullAngle[INWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rm = self.bezierWaypointer4P(1, centAngle, m1, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rb = self.bezierWaypointer4P(2, 2 * np.pi - pushAngle[INWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lb = self.bezierWaypointer4P(3, 2 * np.pi - pushAngle[OUTWARD], m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lm = self.bezierWaypointer4P(4, centAngle, m4, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lf = self.bezierWaypointer4P(5, pullAngle[OUTWARD] / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.right_dominant = rightDominant
            # elif (angularRepresentation[0] < 0.0 and angularRepresentation[1] < 0.0):
            #     self.get_logger().info("Backwards right")
            #     self.leg_waypoints_.rf = self.bezierWaypointer4P(0, np.pi + pushAngle[INWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rm = self.bezierWaypointer4P(1, np.pi - centAngle, m1, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rb = self.bezierWaypointer4P(2, np.pi - pullAngle[INWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lb = self.bezierWaypointer4P(3, np.pi - pullAngle[OUTWARD], m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lm = self.bezierWaypointer4P(4, np.pi - centAngle, m4, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lf = self.bezierWaypointer4P(5, np.pi + pushAngle[OUTWARD] / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.right_dominant = rightDominant
            # else:
            #     self.get_logger().info("Backwards right")
            #     self.leg_waypoints_.rf = self.bezierWaypointer4P(0, np.pi - pushAngle[OUTWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rm = self.bezierWaypointer4P(1, np.pi + centAngle, m1, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.rb = self.bezierWaypointer4P(2, np.pi + pullAngle[OUTWARD], m2, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lb = self.bezierWaypointer4P(3, np.pi + pullAngle[INWARD], m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lm = self.bezierWaypointer4P(4, np.pi + centAngle, m4, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.lf = self.bezierWaypointer4P(5, np.pi - pushAngle[INWARD] / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
            #     self.leg_waypoints_.right_dominant = rightDominant

            if (angularRepresentation[0] > 0.0 and angularRepresentation[1] > 0.0):
                self.get_logger().info("Forward left")
                self.leg_waypoints_.rf = self.bezierWaypointer4P(0, (-2 * tethaOut - tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rm = self.bezierWaypointer4P(1, (-tetha) / 2.0, m4, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rb = self.bezierWaypointer4P(2, (2*tethaOut - tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lb = self.bezierWaypointer4P(3, (2*tethaIn - tetha) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lm = self.bezierWaypointer4P(4, (-tetha) / 2.0, m1, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lf = self.bezierWaypointer4P(5, (-2.0 * tethaIn - tetha)  / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.right_dominant = rightDominant
            elif (angularRepresentation[0] < 0.0 and angularRepresentation[1] > 0.0):
                self.get_logger().info("Forward right")
                self.leg_waypoints_.rf = self.bezierWaypointer4P(0, (2*tethaIn + tetha) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rm = self.bezierWaypointer4P(1, (tetha) / 2.0, m1, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rb = self.bezierWaypointer4P(2, (tetha - 2*tethaIn) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lb = self.bezierWaypointer4P(3, (tetha - 2*tethaOut) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lm = self.bezierWaypointer4P(4, (tetha) / 2.0, m4, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lf = self.bezierWaypointer4P(5, (2*tethaOut + tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.right_dominant = rightDominant
            elif (angularRepresentation[0] < 0.0 and angularRepresentation[1] < 0.0):
                self.get_logger().info("Backwards right")
                self.leg_waypoints_.rf = self.bezierWaypointer4P(0, np.pi - (tetha - 2*tethaIn) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rm = self.bezierWaypointer4P(1, np.pi - (tetha / 2.0), m1, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rb = self.bezierWaypointer4P(2, (2.0 * np.pi - 2.0 * tethaIn - tetha) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lb = self.bezierWaypointer4P(3, (2.0 * np.pi - 2.0 * tethaOut - tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lm = self.bezierWaypointer4P(4, np.pi - (tetha / 2.0), m4, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lf = self.bezierWaypointer4P(5, np.pi - (tetha - 2*tethaOut) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.right_dominant = rightDominant
            else:
                self.get_logger().info("Backwards right")
                self.leg_waypoints_.rf = self.bezierWaypointer4P(0, np.pi + (-2 * tethaOut - tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rm = self.bezierWaypointer4P(1, np.pi + (tetha) / 2.0, m4, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.rb = self.bezierWaypointer4P(2, np.pi + (2*tethaOut - tetha) / 2.0, m3, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lb = self.bezierWaypointer4P(3, np.pi + (2*tethaIn - tetha) / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lm = self.bezierWaypointer4P(4, np.pi + (tetha) / 2.0, m1, gaitAltitude, gaitWidth, rightDominant)
                self.leg_waypoints_.lf = self.bezierWaypointer4P(5, np.pi + (-2.0 * tethaIn - tetha)  / 2.0, m2, gaitAltitude, gaitWidth, rightDominant)
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

    ## Averages two lists of points         
    def pointAverager(self, pointList1, pointList2):
        tempList = []

        for index, point1 in enumerate(pointList1):
            temp = Point()
            temp.x = (point1.x + pointList2[index].x) / 2.0
            temp.y = (point1.y + pointList2[index].y) / 2.0
            temp.z = (point1.z + pointList2[index].z) / 2.0 

            tempList.append(temp)
        self.get_logger().info("A: \n"+str(pointList1) + "\nB: " + str(pointList2) + "\nC: " + str(tempList))
        return tempList

    ## Creates and returns a point from the x y z components
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
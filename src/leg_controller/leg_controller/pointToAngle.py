import rclpy
from rclpy.node import Node
import numpy as np
from time import *
from leg_controller.legInfo import LB, LM, LF, RB, RM, RF, legReferencing
from hexapod_interfaces.msg import TargetAngles, TargetPositions
from geometry_msgs.msg import Point

class inverseKinematics(Node):    
    ## Node Constructor
    def __init__(self):
        # Initialize nodes
        super().__init__("inverse_kinematics_node")
        # Used for transmiting the servo target angles to the servo node
        self.targetAngles = TargetAngles()

        # Declaring the running parameters with their default values
        self.declare_parameter("coxa_len", 71.5)
        self.declare_parameter("femur_len", 100.02)
        self.declare_parameter("tibia_len", 150.0)
        self.declare_parameter("gait_altitude", 90.0)
        self.declare_parameter("base_width", 65.0)

        # Subscribing to the target potisions and publishing the target angles
        self.posSub = self.create_subscription(TargetPositions, 'HexLegPos', self.posCallback, 10)
        self.angles = self.create_publisher(TargetAngles, 'HexAngles', 10)

    def setPoint(self, xT = float, yT = float, zT = float):
        tempPoint = Point()
        tempPoint.x = float(xT)
        tempPoint.y = float(yT)
        tempPoint.z = float(zT)

        return tempPoint

    ## This function takes the position subscription and publishes the computed angles
    def posCallback(self, hexPos = TargetPositions):
        self.getLegAngles(self.setPoint(hexPos.x_pos[1], hexPos.y_pos[1], hexPos.z_pos[1]), RF)
        self.getLegAngles(self.setPoint(hexPos.x_pos[2], hexPos.y_pos[2], hexPos.z_pos[2]), RM)
        self.getLegAngles(self.setPoint(hexPos.x_pos[3], hexPos.y_pos[3], hexPos.z_pos[3]), RB)
        self.getLegAngles(self.setPoint(hexPos.x_pos[4], hexPos.y_pos[4], hexPos.z_pos[4]), LB)
        self.getLegAngles(self.setPoint(hexPos.x_pos[5], hexPos.y_pos[5], hexPos.z_pos[5]), LM)
        self.getLegAngles(self.setPoint(hexPos.x_pos[6], hexPos.y_pos[6], hexPos.z_pos[6]), LF)

        self.angles.publish(self.targetAngles)

    ## Function solving the inverse kinematic model for a given specific leg and point 
    def getLegAngles(self, point = Point, leg = legReferencing, publish = False):
        # Getting the defined constants from the parameters
        coxa_len_ = self.get_parameter("coxa_len").value
        femur_len_ = self.get_parameter("femur_len").value
        tibia_len_ = self.get_parameter("tibia_len").value
        gait_altitude_ = self.get_parameter("gait_altitude").value
        base_width_ = self.get_parameter("base_width").value

        self.get_logger().info("Leg origin: " + str(leg.origin_x) + " | " + str(leg.origin_y))
        # Distance from the leg origin to the point in a planar view
        D = np.sqrt((point.x - leg.origin_x) * (point.x - leg.origin_x) + (point.y - leg.origin_y) * (point.y - leg.origin_y))
        # Distance without the coxa
        L = D - coxa_len_
        # Triangle height from the input point and the desired gait altitude
        A = point.z - gait_altitude_
    
        # Distance the origin to the tip of the leg
        Lprime = np.sqrt(A * A + L * L)
        self.get_logger().info("Kin Distances: " + str(D) + " | " + str(L) + " | " + str(Lprime))

        # (x, y) plane. z is projected on this plane
        T = np.sqrt(point.x * point.x + point.y * point.y)
        alphaPrime = np.arccos(self.limiter((D * D + base_width_ * base_width_ - T * T) / (2 * D * base_width_), -1.0, 1.0))
        alpha = self.limiter(np.rad2deg(alphaPrime) - 90.0)

        # (z, D) plane. x and y are projected on the D-plane
        delta = np.arcsin(L / Lprime)
        beta = np.arccos(self.limiter((tibia_len_ * tibia_len_ - femur_len_ * femur_len_ - Lprime * Lprime) / (-2.0 * femur_len_ * Lprime), -1.0, 1.0))
        sigma = np.arccos(self.limiter((Lprime * Lprime - femur_len_ * femur_len_ - tibia_len_ * tibia_len_) / (-2.0 * femur_len_ * tibia_len_), -1.0, 1.0))

        # There are two cases as the triangle switches side when the leg goes over the robot's base level
        tetha1 = self.limiter(180.0 - np.rad2deg(beta + delta) if point.z < gait_altitude_ else np.rad2deg(delta - beta))
        tetha2 = self.limiter(90.0 + (180.0 - (np.rad2deg(sigma) + 45.0))) # Note that the 45 represents the assembly offset between the tibia and femur!

        self.get_logger().info("Angles: " + str(alpha) + " | " + str(tetha1) + " | " + str(tetha2) + "\n")

        # Setting the target angles for the leg the et_parinverse kinematics were computed for
        self.targetAngles.shoulder_angle[leg.index] = alpha
        self.targetAngles.hip_angle[leg.index] = tetha1
        self.targetAngles.knee_angle[leg.index] = tetha2

        # If we want to publish from here, publish must be set to true
        if (publish == True):
            self.angles.publish(self.targetAngles)

    ## Simple function to set the limits of the angles for the servos in order to not get out of bounds or to limit the servo movement. Also used to keep trig values within ranges
    def limiter(self, value, minLimit = 0.0, maxLimit = 180.0):
        if value > maxLimit:
            value = maxLimit
        if value < minLimit:
            value = minLimit

        return value

def main(args = None):
    rclpy.init(args = args)
    invKinNode = inverseKinematics()
    
    # test height movement
    #for heigth in range(0,250):
    #    invKinNode.getLegAngles(Point(200, 200, heigth), RF)
    #    sleep(0.01)

    #for length in range(150,250):
    #    invKinNode.getLegAngles(Point(length, 200, 0), RF)
    #    sleep(0.01)

    #for width in range(150,250):
    #    invKinNode.getLegAngles(Point(200, width, 0), RF)
    #    sleep(0.01)

    rclpy.spin(invKinNode)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

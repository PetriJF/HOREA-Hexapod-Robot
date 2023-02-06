import rclpy
from rclpy.node import Node
import numpy as np
from time import *
from leg_controller.legInfo import LB, LM, LF, RB, RM, RF, Point, legReferencing
from hexapod_interfaces.msg import TargetAngles

class inverseKinematics(Node):    
    def __init__(self):
        super().__init__("inverse_kinematics_node")
        self.targetAngles = TargetAngles()
        self.angles = self.create_publisher(TargetAngles, 'HexAngles', 10)

    def getLegAngles(self, point = Point, leg = legReferencing):
        # Constants
        COXA_LEN = 71.5
        FEMUR_LEN = 100.02
        FOOT_HEIGHT = 0
        TIBIA_LEN = 150 + FOOT_HEIGHT
        GAIT_ALTITUDE = 90
        BASE_WIDTH = 65.0 # mm

        self.get_logger().info("Leg origin: " + str(leg.originX) + " | " + str(leg.originY))
        # Distance from the leg origin to the point in a planar view
        D = np.sqrt((point.x - leg.originX) * (point.x - leg.originX) + (point.y - leg.originY) * (point.y - leg.originY))
        # Distance without the coxa
        L = D - COXA_LEN
        # Triangle height from the input point and the desired gait altitude
        A = point.z - GAIT_ALTITUDE
    
        # Distance the origin to the tip of the leg
        Lprime = np.sqrt(A * A + L * L)
        self.get_logger().info("Kin Distances: " + str(D) + " | " + str(L) + " | " + str(Lprime))

        # (x, y) plane. z is projected on this plane
        T = np.sqrt(point.x * point.x + point.y * point.y)
        alphaPrime = np.arccos((D * D + BASE_WIDTH * BASE_WIDTH - T * T) / (2 * D * BASE_WIDTH))
        alpha = (np.rad2deg(alphaPrime) - 90.0)

        # (z, D) plane. x and y are projected on the D-plane
        delta = np.arcsin(L / Lprime)
        beta = np.arccos((TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN - Lprime * Lprime) / (-2.0 * FEMUR_LEN * Lprime))
        sigma = np.arccos((Lprime * Lprime - FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN) / (-2.0 * FEMUR_LEN * TIBIA_LEN))

        # There are two cases as the triangle switches side when the leg goes over the robot's base level
        tetha1 = 180.0 - np.rad2deg(beta + delta) if point.z < GAIT_ALTITUDE else np.rad2deg(delta - beta)
        tetha2 = 90.0 + (180.0 - (np.rad2deg(sigma) + 45.0)) # Note that the 45 represents the assembly offset between the tibia and femur!

        self.get_logger().info("Angles: " + str(alpha) + " | " + str(tetha1) + " | " + str(tetha2) + "\n")

        self.targetAngles.shoulder_angle[leg.index] = alpha
        self.targetAngles.hip_angle[leg.index] = tetha1
        self.targetAngles.knee_angle[leg.index] = tetha2

        self.angles.publish(self.targetAngles)

def main(args = None):
    rclpy.init(args = args)
    invKinNode = inverseKinematics()
    
    # test height movement
    for heigth in range(0,250):
        invKinNode.getLegAngles(Point(200, 200, heigth), RF)
        sleep(0.05)

    #for length in range(150,250):
    #    invKinNode.getLegAngles(Point(length, 200, 0), RF)
    #    sleep(0.05)

    #for width in range(150,250):
    #    invKinNode.getLegAngles(Point(200, width, 0), RF)
    #    sleep(0.05)

    #invKinNode.getLegAngles(Point(180, 150, 0), RF)
    #sleep(2.0)
    #invKinNode.getLegAngles(Point(180, 180, 0), RF)
    #sleep(2.0)
    #invKinNode.getLegAngles(Point(180, 210, 0), RF)
    #sleep(2.0)
    #invKinNode.getLegAngles(Point(180, 240, 0), RF)
    #sleep(2.0)
    #invKinNode.getLegAngles(Point(135, 135, 30), RF)
    #sleep(2.0)
    #invKinNode.getLegAngles(Point(135, 135, 60), RF)
    #sleep(2.0)

    #rclpy.spin(invKinNode)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


    #alpha = np.arctan(np.absolute(point.y - HIP_ORIGIN.y) / (L))
    #beta = np.arccos((D * D + FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN) / (2 * FEMUR_LEN * D))
    #gamma = np.arccos((FEMUR_LEN *FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - D * D) / (2 * FEMUR_LEN * TIBIA_LEN))
    #print(alpha, beta, gamma, np.degrees(alpha), np.degrees(beta), np.degrees(beta-alpha))

    #tetha = np.degrees(np.arctan((point.z - HIP_ORIGIN.z) / (point.x - HIP_ORIGIN.x)))
    #tetha1 = 180 - np.degrees(beta + alpha)
    #tetha2 = 180 - np.degrees(gamma)
    #tetha = np.degrees(np.arctan((point.z - HIP_ORIGIN.z) / (point.x - HIP_ORIGIN.x)))
    #tetha1 = np.degrees(np.pi / 2 - (np.arccos((TIBIA_LEN*TIBIA_LEN - FEMUR_LEN*FEMUR_LEN - L2*L2) / (-2 * FEMUR_LEN * L2)) - alpha))
    #tetha2 =  np.degrees(np.pi -np.arccos((L2*L2 - FEMUR_LEN*FEMUR_LEN - TIBIA_LEN*TIBIA_LEN) / (-2 * FEMUR_LEN * TIBIA_LEN)))

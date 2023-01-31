import rclpy
from rclpy.node import Node
import numpy as np
from leg_controller.legInfo import Point, Angles, legReferencing

class inverseKinematics(Node):    
    def __init__(self):
        super().__init__("inverse_kinematics_node")
        
    def getLegAngles(self, point = Point, leg = legReferencing):
        # Constants
        COXA_LEN = 71.5
        FEMUR_LEN = 100.02
        FOOT_HEIGHT = 0
        TIBIA_LEN = 150 + FOOT_HEIGHT
        GAIT_ALTITUDE = 90
        BASE_ALTITUDE = GAIT_ALTITUDE - point.z
        BASE_WIDTH = 65.0 # mm

        # Distance from the leg origin to the point in a planar view
        D = np.sqrt((point.x - leg.originX) * (point.x - leg.originX) + (point.y - leg.originY) * (point.y - leg.originY))
        # Distance without the coxa
        L = D - COXA_LEN
        # Distance the origin to the tip of the leg
        Lprime = np.sqrt(BASE_ALTITUDE * BASE_ALTITUDE + L * L)
        self.get_logger().info(D + " | " + L + " | " + Lprime)

        # (x, y) plane. z is projected on this plane
        T = round(np.sqrt(point.x * point.x + point.y * point.y), 2)
        alphaPrime = np.arccos((D * D + BASE_WIDTH * BASE_WIDTH - T * T) / (2 * D * BASE_WIDTH))
        alpha = 180.0 - round(np.rad2deg(alphaPrime), 2)

        # (z, D) plane. x and y are projected on the D-plane
        delta = round(np.arctan(BASE_ALTITUDE / L), 2)
        beta = round(np.arccos((TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN - Lprime * Lprime) / (-2.0 * FEMUR_LEN * Lprime)), 2)
        sigma = round(np.arccos((Lprime * Lprime - FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN) / (-2.0 * FEMUR_LEN * TIBIA_LEN)), 2)

        tetha1 = np.rad2deg(beta - delta)
        tetha2 = 180.0 - np.rad2deg(sigma) - 45.0 # Note that the 45 represents the assembly offset between the tibia and femur!

        self.get_logger().info(alpha + " | " + tetha1 + " | " + tetha2)
        return Angles(alpha, tetha1, tetha2)








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

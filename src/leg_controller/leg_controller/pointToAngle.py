import rclpy
from rclpy.node import Node
import numpy as np
from time import *
from hexapod_interfaces.msg import TargetAngles, TargetPositions
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

class inverseKinematics(Node):    
    ## Node Constructor
    def __init__(self):
        # Initialize nodes
        super().__init__("kin_node")
        # Used for transmiting the servo target angles to the servo node
        self.targetAngles = TargetAngles()
        self.currentPos = TargetPositions()
        self.xAxisInclination = 0.0 
        self.yAxisInclination = 0.0
        self.ratio = 40.0

        # Declaring the robot design paramteres. Note 3 = PARAMETER_DOUBLE
        pd = ParameterDescriptor(description = "Robot Structure Description", type = 3) 
        self.declare_parameter(name = "coxa_len", descriptor = pd)
        self.declare_parameter(name = "femur_len", descriptor = pd)
        self.declare_parameter(name = "tibia_len", descriptor = pd)
        self.declare_parameter(name = "base_altitude", descriptor = pd)
        self.declare_parameter(name = "base_width", descriptor = pd)
        
        # Declaring the parameters representing each leg origin. Note 8 = PARAMETER_DOUBLE_ARRAY
        pd = ParameterDescriptor(description = "Origin definition for each leg", type = 8) 
        self.declare_parameter(name = "origin_RF", descriptor = pd)
        self.declare_parameter(name = "origin_RM", descriptor = pd)
        self.declare_parameter(name = "origin_RB", descriptor = pd)
        self.declare_parameter(name = "origin_LB", descriptor = pd)
        self.declare_parameter(name = "origin_LM", descriptor = pd)
        self.declare_parameter(name = "origin_LF", descriptor = pd)

        # Declaring the parameter representing the default orientation of each leg. Note 8 = PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(name = "leg_angular_orientation", descriptor = pd)

        # Initializing the robot design variables from the parameters
        self.coxa_len_ = self.get_parameter("coxa_len").value
        self.femur_len_ = self.get_parameter("femur_len").value
        self.tibia_len_ = self.get_parameter("tibia_len").value
        self.base_altitude_ = self.get_parameter("base_altitude").value
        self.base_width_ = self.get_parameter("base_width").value


        # Initializing the origins from the parameters as point to make it more readable through the code.
        self.origin_RF_ = self.setPoint(self.get_parameter("origin_RF").value[0], self.get_parameter("origin_RF").value[1], 0.0)
        self.origin_RM_ = self.setPoint(self.get_parameter("origin_RM").value[0], self.get_parameter("origin_RM").value[1], 0.0)
        self.origin_RB_ = self.setPoint(self.get_parameter("origin_RB").value[0], self.get_parameter("origin_RB").value[1], 0.0)
        self.origin_LB_ = self.setPoint(self.get_parameter("origin_LB").value[0], self.get_parameter("origin_LB").value[1], 0.0)
        self.origin_LM_ = self.setPoint(self.get_parameter("origin_LM").value[0], self.get_parameter("origin_LM").value[1], 0.0)
        self.origin_LF_ = self.setPoint(self.get_parameter("origin_LF").value[0], self.get_parameter("origin_LF").value[1], 0.0)

        self.get_logger().info("Parameters declared and initialized successfully!")
        
        # Subscribing to the target potisions and publishing the target angles
        self.posSub = self.create_subscription(TargetPositions, 'HexLegPos', self.posCallback, 10)
        self.baseInclinationSub_ = self.create_subscription(Float64MultiArray, 'base_inclination', self.inclinationCallback, 10)
        self.angles = self.create_publisher(TargetAngles, 'HexAngles', 10)

        
        
        # Note gamma_ values : 0 RF, 1 RM, 2 RB, 3 LB, 4 LM, 5 LF 
        self.gamma_ = np.deg2rad(self.get_parameter("leg_angular_orientation").value)

        # Code used to make a rectangle with the leg. Used to test the accuracy of the kinematic model directly
        #sleep(15.0)
        
        #for leg_index in range(1, 6+1):
        #    self.targetAngles.shoulder_angle[leg_index] = 90.0
        #    self.targetAngles.hip_angle[leg_index] = 90.0
        #    self.targetAngles.knee_angle[leg_index] = 90.0

        #self.get_logger().info("cmd sent")
        #self.getLegAngles(self.setPoint(251.14, 145.0, 0.0), self.origin_RF_, 1)
        #self.angles.publish(self.targetAngles)
        #sleep(5.0)
        #self.getLegAngles(self.setPoint(301.14, 145.0, 0.0), self.origin_RF_, 1)
        #self.angles.publish(self.targetAngles)
        #sleep(5.0)
        #self.getLegAngles(self.setPoint(301.14, 205.0, 0.0), self.origin_RF_, 1)
        #self.angles.publish(self.targetAngles)
        #sleep(5.0)
        #self.getLegAngles(self.setPoint(251.14, 205.0, 0.0), self.origin_RF_, 1)
        #self.angles.publish(self.targetAngles)
        #sleep(5.0)
        #self.getLegAngles(self.setPoint(0.0, 300.0, 0.0), self.origin_RM_, 2)
        #self.getLegAngles(self.setPoint(-251.14, 145.0, 0.0), self.origin_RB_, 3)
        #self.getLegAngles(self.setPoint(-251.14, -145.0, 0.0), self.origin_LB_, 4)
        #self.getLegAngles(self.setPoint(0.0, -300.0, 0.0), self.origin_LM_, 5)
        #self.getLegAngles(self.setPoint(251.14, -145.0, 0.0), self.origin_LF_, 6, publish = True)


        

    ## Gets three values and makes them into a Point
    def setPoint(self, xT = float, yT = float, zT = float):
        tempPoint = Point()
        tempPoint.x = float(xT)
        tempPoint.y = float(yT)
        tempPoint.z = float(zT)
        
        return tempPoint

    ## This function takes the position subscription and publishes the computed angles
    def posCallback(self, hexPos = TargetPositions):
        self.currentPos = hexPos
        if hexPos.z_pos[0] != 0.0:
            self.base_altitude_ = hexPos.z_pos[0] 

        originSet = self.getPlanarOrigins(self.xAxisInclination, self.yAxisInclination)

        self.getLegAngles(self.setPoint(hexPos.x_pos[1], hexPos.y_pos[1], hexPos.z_pos[1]), originSet[0], 1)
        self.getLegAngles(self.setPoint(hexPos.x_pos[2], hexPos.y_pos[2], hexPos.z_pos[2]), originSet[1], 2)
        self.getLegAngles(self.setPoint(hexPos.x_pos[3], hexPos.y_pos[3], hexPos.z_pos[3]), originSet[2], 3)
        self.getLegAngles(self.setPoint(hexPos.x_pos[4], hexPos.y_pos[4], hexPos.z_pos[4]), originSet[3], 4)
        self.getLegAngles(self.setPoint(hexPos.x_pos[5], hexPos.y_pos[5], hexPos.z_pos[5]), originSet[4], 5)
        self.getLegAngles(self.setPoint(hexPos.x_pos[6], hexPos.y_pos[6], hexPos.z_pos[6]), originSet[5], 6)

        self.angles.publish(self.targetAngles)

    ## Function solving the inverse kinematic model for a given specific leg and point 
    def getLegAngles(self, point = Point, origin = Point, leg_index = int, publish = False):
        # Distance from the leg origin to the point in a planar view
        D = np.sqrt((point.x - origin.x) * (point.x - origin.x) + (point.y - origin.y) * (point.y - origin.y))
        # Distance without the coxa
        L = D - self.coxa_len_
        # Triangle height from the input point and the desired gait altitude
        # Note: The last term represents the tilt modification on the specific leg
        A = point.z - self.base_altitude_ - origin.z
        # Distance the origin to the tip of the leg
        Lprime = np.sqrt(A * A + L * L)
        
        # Get the step displacement from the home location
        T = np.sqrt((point.x - origin.x - D * np.cos(self.gamma_[leg_index - 1])) ** 2 + (point.y - origin.y -  D * np.sin(self.gamma_[leg_index - 1])) ** 2)
        # Get the shoulder angle relative to the middle line
        alphaPrime = np.rad2deg(np.arccos((2 * D * D - T * T) / (2 * D * D)))
        # Because cos is sign dependent, ro is used to determine on which side of the default line the leg goes
        ro = np.arccos(point.x / (np.sqrt(point.x * point.x + point.y * point.y)))
        if point.y < 0: # If we are working with legs from the left side of the robot, we must change the rotation direction and substract from 360
            ro = 2 * np.pi - ro
        # Based on the side of the point compared to the center rest position, add or substract from 90 alphaPrime, giving the final value of alpha
        alpha = 90.0 + (alphaPrime if (ro < self.gamma_[leg_index - 1]) else (-alphaPrime))
        # Line used to debug kinematic model.
        #self.get_logger().info(str(leg_index) + "->" + str(alpha) + " for " + str(point.x) + ", " + str(point.y) + " and " + str(origin.x) + " " + str(origin.y) + "\nD = " + str(D) + " T = " + str(T) + " aprim = " + str(alphaPrime))

        # (z, D) plane. x and y are projected on the D-plane
        delta = np.arcsin(L / Lprime)
        beta = np.arccos(self.limiter((self.tibia_len_ * self.tibia_len_ - self.femur_len_ * self.femur_len_ - Lprime * Lprime) / (-2.0 * self.femur_len_ * Lprime), -1.0, 1.0))
        sigma = np.arccos(self.limiter((Lprime * Lprime - self.femur_len_ * self.femur_len_ - self.tibia_len_ * self.tibia_len_) / (-2.0 * self.femur_len_ * self.tibia_len_), -1.0, 1.0))

        # There are two cases as the triangle switches side when the leg goes over the robot's base level
        tetha1 = self.limiter(180.0 - np.rad2deg(beta + delta) if point.z < self.base_altitude_ else np.rad2deg(delta - beta))
        tetha2 = self.limiter(90.0 + (180.0 - (np.rad2deg(sigma) + 45.0))) # Note that the 45 represents the assembly offset between the tibia and femur!

        # Setting the target angles for the leg the et_parinverse kinematics were computed for
        self.targetAngles.shoulder_angle[leg_index] = alpha
        self.targetAngles.hip_angle[leg_index] = tetha1
        self.targetAngles.knee_angle[leg_index] = tetha2

        # If we want to publish from here, publish must be set to true
        if (publish == True):
            self.angles.publish(self.targetAngles)

    ## Computes the origins of the legs based on the two inclination variables
    def getPlanarOrigins(self, xTilt = float, yTilt = float):
        return [
            self.getIndexPlanarOrigin(self.origin_RF_, xTilt, yTilt, 0),
            self.getIndexPlanarOrigin(self.origin_RM_, xTilt, yTilt, 1),
            self.getIndexPlanarOrigin(self.origin_RB_, xTilt, yTilt, 2),
            self.getIndexPlanarOrigin(self.origin_LB_, xTilt, yTilt, 3),
            self.getIndexPlanarOrigin(self.origin_LM_, xTilt, yTilt, 4),
            self.getIndexPlanarOrigin(self.origin_LF_, xTilt, yTilt, 5)
        ]
    
    def getIndexPlanarOrigin(self, origin = Point(), xTilt = float, yTilt = float, index = int):
        temp = Point()
        # ground plane 0 axis
        temp.x = origin.x * np.cos(yTilt) + yTilt * self.ratio
        # ground plane
        temp.y = origin.y * np.cos(xTilt) + xTilt * self.ratio
        # depth axis
        temp.z = origin.x * np.sin(yTilt) + origin.y * np.sin(xTilt)

        return temp

    ## Simple function to set the limits of the angles for the servos in order to not get out of bounds or to limit the servo movement. Also used to keep trig values within ranges
    def limiter(self, value, minLimit = 0.0, maxLimit = 180.0):
        if value > maxLimit:
            value = maxLimit
        if value < minLimit:
            value = minLimit

        return value

    def inclinationCallback(self, inclinations):
        self.xAxisInclination = inclinations.data[0] 
        self.yAxisInclination = inclinations.data[1]

        self.posCallback(self.currentPos)

def main(args = None):
    # Initialize the node
    rclpy.init(args = args)
    invKinNode = inverseKinematics()
    
    # Keep the node running
    try:
        rclpy.spin(invKinNode)
    except KeyboardInterrupt:
        pass

    invKinNode.destroy_node()
    # Stop the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

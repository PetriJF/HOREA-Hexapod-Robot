#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import *
import numpy as np
from hexapod_interfaces.msg import TargetPositions

## Node use for the control strategy of the robot
class ControlNode(Node):
    ## Node Constructor
    def __init__(self):
        super().__init__("ControlNode")
        self.targetPositions = TargetPositions()
        self.hexPositions = self.create_publisher(TargetPositions, 'HexLegPos', 10)

    ## Sets the legs to the initial position of having all the legs straight up.
    def hexInitPosition(self):
        COXA_LEN = 71.5
        BASE_WIDTH = 65.0
        FEMUR_LEN = 100.02
        FOOT_HEIGHT = 0
        TIBIA_LEN = 150 + FOOT_HEIGHT
        GAIT_HEIGHT = 90.0


        # Getting the radius at which we want the legs to be at
        INIT_RADIUS = COXA_LEN + BASE_WIDTH
        # Getting the height of the init position
        H = FEMUR_LEN + TIBIA_LEN + GAIT_HEIGHT

        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     INIT_RADIUS * np.sin(np.pi/6.0),
                                     INIT_RADIUS * np.sin(np.pi/2.0),
                                     INIT_RADIUS * np.sin(5.0*np.pi/6.0),
                                     INIT_RADIUS * np.sin(7.0*np.pi/6.0),
                                     INIT_RADIUS * np.sin(3.0*np.pi/2.0),
                                     INIT_RADIUS * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     INIT_RADIUS * np.cos(np.pi/6.0),
                                     INIT_RADIUS * np.cos(np.pi/2.0),
                                     INIT_RADIUS * np.cos(5.0*np.pi/6.0),
                                     INIT_RADIUS * np.cos(7.0*np.pi/6.0),
                                     INIT_RADIUS * np.cos(3.0*np.pi/2.0),
                                     INIT_RADIUS * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
        #self.get_logger().info(str(self.targetPositions))

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexZeroPosition(self):
        GAIT_WIDTH = 300.0
        GAIT_HEIGHT = 90.0
        H = GAIT_HEIGHT

        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     GAIT_WIDTH * np.sin(np.pi/6.0),
                                     GAIT_WIDTH * np.sin(np.pi/2.0),
                                     GAIT_WIDTH * np.sin(5.0*np.pi/6.0),
                                     GAIT_WIDTH * np.sin(7.0*np.pi/6.0),
                                     GAIT_WIDTH * np.sin(3.0*np.pi/2.0),
                                     GAIT_WIDTH * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     GAIT_WIDTH * np.cos(np.pi/6.0),
                                     GAIT_WIDTH * np.cos(np.pi/2.0),
                                     GAIT_WIDTH * np.cos(5.0*np.pi/6.0),
                                     GAIT_WIDTH * np.cos(7.0*np.pi/6.0),
                                     GAIT_WIDTH * np.cos(3.0*np.pi/2.0),
                                     GAIT_WIDTH * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
        #self.get_logger().info(str(self.targetPositions))

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexStandPosition(self, raiseTime = 5.0, raiseResolution = 1.0):
        GAIT_WIDTH = 200.0
        GAIT_HEIGHT = 90.0

        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     GAIT_WIDTH * np.sin(np.pi/6.0),
                                     GAIT_WIDTH * np.sin(np.pi/2.0),
                                     GAIT_WIDTH * np.sin(5.0*np.pi/6.0),
                                     GAIT_WIDTH * np.sin(7.0*np.pi/6.0),
                                     GAIT_WIDTH * np.sin(3.0*np.pi/2.0),
                                     GAIT_WIDTH * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     GAIT_WIDTH * np.cos(np.pi/6.0),
                                     GAIT_WIDTH * np.cos(np.pi/2.0),
                                     GAIT_WIDTH * np.cos(5.0*np.pi/6.0),
                                     GAIT_WIDTH * np.cos(7.0*np.pi/6.0),
                                     GAIT_WIDTH * np.cos(3.0*np.pi/2.0),
                                     GAIT_WIDTH * np.cos(11.0*np.pi/6.0)
        ]

        for height in np.arange(0.0, GAIT_HEIGHT + 1, raiseResolution):
            H = GAIT_HEIGHT - height
            self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
            self.hexPositions.publish(self.targetPositions)
            sleep(raiseTime / ((1.0 / raiseResolution) * (GAIT_HEIGHT + 1)))


def main(args = None):
    rclpy.init(args = args)
    ctrl = ControlNode()
    
    ctrl.hexInitPosition()
    sleep(5.0)
    ctrl.hexZeroPosition()
    sleep(2.0)
    ctrl.hexStandPosition()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
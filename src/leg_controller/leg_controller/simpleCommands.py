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

    def hexStandPosition(self, raiseTime = 5.0, raiseResolution = 1.0, lower = False):
        GAIT_WIDTH = 300.0
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
            # if lower is true, we are ascending the legs, if not, descending
            H = height if (lower == True) else GAIT_HEIGHT - height

            # set the z positions of the legs and publish them
            self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
            self.hexPositions.publish(self.targetPositions)

            # control the speed of the standing transition
            sleep(raiseTime / ((1.0 / raiseResolution) * (GAIT_HEIGHT + 1)))


def main(args = None):
    rclpy.init(args = args)
    ctrl = ControlNode()
    
    print("List of commands:\n\t1. Legs Up\n\t2. Legs Preped\n\t3. Raise Base\n\t4. Lower Base\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    
    userInput = input("\nEnter command: ")

    while userInput != 'c':
        if (userInput == '1'):
            ctrl.hexInitPosition()
            print("Legs Up Pose")
        elif (userInput == '2'):
            ctrl.hexZeroPosition()
            print("Legs userInput Stand Prep Pose")
        elif (userInput == '3'):
            ctrl.hexStandPosition(raiseTime = 1.0, raiseResolution = 2.0)
            print("Robot Raising!!")
        elif (userInput == '4'):
            ctrl.hexStandPosition(raiseTime = 2.5, lower = True)
            print("Robot Lowering!!")
        else:
            print("Wrong command")
        
        userInput = input("\nEnter command: ")
     
    rclpy.shutdown()

if __name__ == '__main__':
    main()
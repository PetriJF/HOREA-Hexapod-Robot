#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import *
import numpy as np
from hexapod_interfaces.msg import TargetPositions
from std_msgs.msg import String

## Node use for the control strategy of the robot
class ControlNode(Node):
    ## Node Constructor
    def __init__(self):
        super().__init__("ControlNode")

        self.declare_parameter("coxa_len", 71.5)
        self.declare_parameter("femur_len", 100.02)
        self.declare_parameter("tibia_len", 150.0)
        self.declare_parameter("gait_altitude", 90.0)
        self.declare_parameter("base_width", 65.0)
        self.declare_parameter("gait_width", 300.0)

        self.coxa_len_ = self.get_parameter("coxa_len").value
        self.femur_len_ = self.get_parameter("femur_len").value
        self.tibia_len_ = self.get_parameter("tibia_len").value
        self.gait_altitude_ = self.get_parameter("gait_altitude").value
        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value

        self.targetPositions = TargetPositions()
        self.posSub = self.create_subscription(String, 'legsCommand', self.legsCommand, 10)
        self.hexPositions = self.create_publisher(TargetPositions, 'HexLegPos', 10)

    ## TODO ADD COMMENTS
    def legsCommand(self, cmd = String):
        if cmd == "1":
            self.hexInitPosition()
            print("Legs Up Pose")
        elif cmd == "2":
            self.hexZeroPosition()
            print("Legs Stand Prep Pose")
        elif cmd == "3":
            self.hexStandPosition(raiseTime = 1.0, raiseResolution = 2.0, lower = False)
            print("Robot Raising!!")
        elif cmd == "4":
            self.hexStandPosition(raiseTime = 2.5, raiseResolution = 1.0, lower = True)
            print("Robot Lowering!!")
        else:
            self.get_logger().warning("Incorrect command sent to the step controller!!")


    ## Sets the legs to the initial position of having all the legs straight up.
    def hexInitPosition(self):
        # Getting the radius at which we want the legs to be at
        INIT_RADIUS = self.coxa_len_ + self.base_width_
        # Getting the height of the init position
        H = self.femur_len_ + self.tibia_len_ + self.gait_altitude_

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

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexZeroPosition(self):
        self.gait_width_ = 300.0
        self.gait_altitude_ = 90.0
        H = self.gait_altitude_

        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     self.gait_width_ * np.sin(np.pi/6.0),
                                     self.gait_width_ * np.sin(np.pi/2.0),
                                     self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                     self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     self.gait_width_ * np.cos(np.pi/6.0),
                                     self.gait_width_ * np.cos(np.pi/2.0),
                                     self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                     self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexStandPosition(self, raiseTime = 5.0, raiseResolution = 1.0, lower = False):
        self.gait_width_ = 300.0
        self.gait_altitude_ = 90.0

        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     self.gait_width_ * np.sin(np.pi/6.0),
                                     self.gait_width_ * np.sin(np.pi/2.0),
                                     self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                     self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     self.gait_width_ * np.cos(np.pi/6.0),
                                     self.gait_width_ * np.cos(np.pi/2.0),
                                     self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                     self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]

        for height in np.arange(0.0, self.gait_altitude_ + 1, raiseResolution):
            # if lower is true, we are ascending the legs, if not, descending
            H = height if (lower == True) else self.gait_altitude_ - height

            # set the z positions of the legs and publish them
            self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
            self.hexPositions.publish(self.targetPositions)

            # control the speed of the standing transition
            sleep(raiseTime / ((1.0 / raiseResolution) * (self.gait_altitude_ + 1)))


def main(args = None):
    rclpy.init(args = args)
    ctrl = ControlNode()
    
    print("List of commands:\n\t1. Legs Up\n\t2. Legs Preped\n\t3. Raise Base\n\t4. Lower Base\n\nType just the number for the action you want\n\nType 'c' to stop\n\n")
    
    userInput = input("\nEnter command: ")
    rclpy.spin(ctrl)

    while userInput != 'c':
        if (userInput == '1'):
            ctrl.hexInitPosition()
            print("Legs Up Pose")
        elif (userInput == '2'):
            ctrl.hexZeroPosition()
            print("Legs Stand Prep Pose")
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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import *
import numpy as np
from hexapod_interfaces.msg import TargetPositions
from std_msgs.msg import Int64MultiArray, Float64

## Node use for the control strategy of the robot
class ControlNode(Node):
    ## Node Constructor
    def __init__(self):
        super().__init__("ControlNode")

        self.get_logger().info("Init")

        self.resolution_ = 0.01
        self.robot_iteration_speed = (0.5 * 1.0 * self.resolution_)

        self.declare_parameter("coxa_len", 71.5)
        self.declare_parameter("femur_len", 100.02)
        self.declare_parameter("tibia_len", 194.051)
        self.declare_parameter("base_altitude", 90.0)
        self.declare_parameter("base_width", 65.0)
        self.declare_parameter("gait_width", 300.0)

        self.coxa_len_ = self.get_parameter("coxa_len").value
        self.femur_len_ = self.get_parameter("femur_len").value
        self.tibia_len_ = self.get_parameter("tibia_len").value
        self.base_altitude_ = self.get_parameter("base_altitude").value
        self.base_width_ = self.get_parameter("base_width").value
        self.gait_width_ = self.get_parameter("gait_width").value

        self.targetPositions = TargetPositions()
        self.posSub = self.create_subscription(Int64MultiArray, 'animationType', self.legsCommand, 10)
        self.speedSub_ = self.create_subscription(Float64, 'step_speed', self.robotSpeedCallback, 10)
        self.hexPositions = self.create_publisher(TargetPositions, 'HexLegPos', 10)

    ## TODO ADD COMMENTS
    def legsCommand(self, cmd):
        if cmd.data[0] == 1:
            self.hexInitPosition()
            print("Legs Up Pose")
        elif cmd.data[0] == 2:
            self.hexZeroPosition()
            print("Legs Stand Prep Pose")
        elif cmd.data[0] == 3:
            self.hexStandPosition(raiseTime = 1.0, raiseResolution = 2.0, lower = False)
            print("Robot Raising!!")
        elif cmd.data[0] == 4:
            self.hexStandPosition(raiseTime = 2.5, raiseResolution = 1.0, lower = True)
            print("Robot Lowering!!")
        elif cmd.data[0] == 10:
            self.hexChangeAltitude(currentAlt = self.base_altitude_, newAlt = cmd.data[1])
        else:
            self.get_logger().warning("Incorrect command sent to the step controller!!")

    def robotSpeedCallback(self, speed):
        self.robot_iteration_speed = (0.5 * speed.data * self.resolution_)

    ## Sets the legs to the initial position of having all the legs straight up.
    def hexInitPosition(self):
        # Getting the height of the init position
        H = 55.0
        
        self.targetPositions.x_pos = [0.0, 
                                     self.gait_width_ * np.cos(np.pi/6.0),
                                     self.gait_width_ * np.cos(np.pi/2.0),
                                     self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                     self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     self.gait_width_ * np.sin(np.pi/6.0),
                                     self.gait_width_ * np.sin(np.pi/2.0),
                                     self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                     self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexZeroPosition(self):
        # Setting the coordingates of all the points
        H = 60.0
        
        self.targetPositions.x_pos = [0.0, 
                                     self.gait_width_ * np.cos(np.pi/6.0),
                                     self.gait_width_ * np.cos(np.pi/2.0),
                                     self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                     self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     self.gait_width_ * np.sin(np.pi/6.0),
                                     self.gait_width_ * np.sin(np.pi/2.0),
                                     self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                     self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]
        self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]

        # Publish the initial position
        self.hexPositions.publish(self.targetPositions)

    def hexStandPosition(self, raiseTime = 5.0, raiseResolution = 1.0, lower = False):
        # Setting the coordingates of all the points
        self.targetPositions.x_pos = [0.0, 
                                     self.gait_width_ * np.cos(np.pi/6.0),
                                     self.gait_width_ * np.cos(np.pi/2.0),
                                     self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                     self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                     self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                     self.gait_width_ * np.sin(np.pi/6.0),
                                     self.gait_width_ * np.sin(np.pi/2.0),
                                     self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                     self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                     self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]

        for height in np.arange(60, self.base_altitude_ + 1, raiseResolution):
            # if lower is true, we are ascending the legs, if not, descending
            H = height - 60.0 if (lower == True) else self.base_altitude_ - height

            # set the z positions of the legs and publish them
            self.targetPositions.z_pos = [ 0.0, H, H, H, H, H, H ]
            self.hexPositions.publish(self.targetPositions)

            # control the speed of the standing transition
            sleep(self.robot_iteration_speed)

    def hexChangeAltitude(self, currentAlt = float, newAlt = int):
        self.base_altitude_ = float(newAlt)
        self.targetPositions.x_pos = [0.0, 
                                    self.gait_width_ * np.cos(np.pi/6.0),
                                    self.gait_width_ * np.cos(np.pi/2.0),
                                    self.gait_width_ * np.cos(5.0*np.pi/6.0),
                                    self.gait_width_ * np.cos(7.0*np.pi/6.0),
                                    self.gait_width_ * np.cos(3.0*np.pi/2.0),
                                    self.gait_width_ * np.cos(11.0*np.pi/6.0)
        ]
        self.targetPositions.y_pos = [0.0, 
                                    self.gait_width_ * np.sin(np.pi/6.0),
                                    self.gait_width_ * np.sin(np.pi/2.0),
                                    self.gait_width_ * np.sin(5.0*np.pi/6.0),
                                    self.gait_width_ * np.sin(7.0*np.pi/6.0),
                                    self.gait_width_ * np.sin(3.0*np.pi/2.0),
                                    self.gait_width_ * np.sin(11.0*np.pi/6.0)
        ]
        direction = 1 if (int(currentAlt) < newAlt) else -1
        for alt in range(int(currentAlt), newAlt + direction, direction):
            H = float(alt)
            self.targetPositions.z_pos = [ H, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
            self.hexPositions.publish(self.targetPositions)
            sleep(self.robot_iteration_speed)

            

def main(args = None):
    rclpy.init(args = args)
    ctrl = ControlNode()
    
    try:
        rclpy.spin(ctrl)
    except KeyboardInterrupt:
        pass
    
    ctrl.get_logger().info("Shutting down..")
    ctrl.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
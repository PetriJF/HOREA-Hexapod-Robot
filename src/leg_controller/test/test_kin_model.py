import os
import sys
import time
import unittest
import uuid
import numpy as np

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
from hexapod_interfaces.msg import LegReferencing
import pytest

import rclpy

# Launch features

@pytest.mark.launch_test
def generate_test_description():
    # Robot constants
    COXA_LEN = 71.5
    FEMUR_LEN = 100.02
    FOOT_HEIGHT = 0.0
    TIBIA_LEN = 150.0 + FOOT_HEIGHT
    BASE_ALTITUDE = 120.0
    BASE_WIDTH = 65.0
    GAIT_ALTITUDE = 120.0
    STEP_LENGTH = 50.0
    GAIT_WIDTH = 290.0
    ANIMATION_RESOLUTION = 0.01
    STEP_DURATION = 1.0
    
    
    file_path = os.path.dirname(__file__)
    
    # Kinematics node
    kin_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments = [ os.path.join(file_path, "..", "leg_controller", "pointToAngle.py")],
        additional_env = {'PYTHONUNBUFFERED': '1'},
        parameters = [
            {"coxa_len": COXA_LEN},
            {"femur_len": FEMUR_LEN},
            {"tibia_len": TIBIA_LEN},
            {"base_altitude": BASE_ALTITUDE},
            {"base_width": BASE_WIDTH},
            {"leg_angular_orientation": [ 30.0, 90.0, 150.0, 210.0, 270.0, 330.0 ]},
            {"origin_RF": [ BASE_WIDTH * round(np.cos(np.pi/6.0),2), BASE_WIDTH * round(np.sin(np.pi/6.0),2) ]},
            {"origin_RM": [ BASE_WIDTH * round(np.cos(np.pi/2.0),2), BASE_WIDTH * round(np.sin(np.pi/2.0),2) ]},
            {"origin_RB": [ BASE_WIDTH * round(np.cos(5.0*np.pi/6.0),2), BASE_WIDTH * round(np.sin(5.0*np.pi/6.0),2) ]},
            {"origin_LB": [ BASE_WIDTH * round(np.cos(7.0*np.pi/6.0),2), BASE_WIDTH * round(np.sin(7*np.pi/6.0),2) ]},
            {"origin_LM": [ BASE_WIDTH * round(np.cos(3.0*np.pi/2.0),2), BASE_WIDTH * round(np.sin(3.0*np.pi/2.0),2) ]},
            {"origin_LF": [ BASE_WIDTH * round(np.cos(11.0*np.pi/6.0),2), BASE_WIDTH * round(np.sin(11.0*np.pi/6.0),2) ]},

            {"topic": "HexLegPos"}
        ]
    )

    # Servo angle node
    servo_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments = [ os.path.join(file_path, "..", "leg_controller", "servoController.py") ],
        additional_env = {'PYTHONUNBUFFERED': '1'},
        parameters = [
            {"LB": [ 8, 9, 10 ]},
            {"LM": [ 4, 5, 6 ]},
            {"LF": [ 0, 1, 2 ]},
            {"RF": [ 8, 9, 10 ]},
            {"RM": [ 4, 5, 6 ]},
            {"RB": [ 0, 1, 2 ]}
        ]
    )

    return (
        launch.LaunchDescription([
            kin_node,
            servo_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest()
        ]), 
        {
            'kin_node': kin_node,
            'servo_node': servo_node
        }
    ) 


# Test features
class TestKinModelServoLink(unittest.TestCase):
    
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from hexapod_interfaces.msg import TargetAngles, TargetPositions
import numpy as np
import os
import subprocess
import time

# Create a test node that will act as the subscriber and publisher
class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.input_sent = False
        self.output_received = False
        self.out = TargetAngles()
        self.subscription = self.create_subscription(TargetAngles, 'HexAngles', self.output_callback, 10)
        self.publisher = self.create_publisher(TargetPositions, 'HexLegPos', 10)

    def input_callback(self, msg):
        # Send the input message to the kin_node for processing
        self.publisher.publish(msg)

    def output_callback(self, msg):
        # Check if the received message matches the expected output
        self.output_received = True
        self.out = msg

# Define the test function
def test_kin_node():
    
    # Preparing the running parameters and the command needed to run the node
    kinParams = os.path.join(get_package_share_directory("hexapod_bringup"), "config", "kinParams.yaml")
    kin_node_cmd = ['ros2', 'run', 'leg_controller', 'kin_node', '--ros-args', '--params-file', kinParams]
    
    # Running the node as a subprocess
    kin_process = subprocess.Popen(kin_node_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Wait for the node to start up
    time.sleep(5)

    # Check if the node started successfully
    assert kin_process.poll() is None, f"Kin node failed to start. Error: {kin_process.stderr.read().decode()}"
    
    # Initialize the ROS 2 context
    rclpy.init()

    # Create a test node that will act as the subscriber and publisher
    test_node = TestNode()

    # Send a custom input message to the kin_node
    input_msg = TargetPositions()
    robotReach = 10.0 + 65.0 + 71.5 + 100.02 + 150.0
    input_msg = TargetPositions()
    input_msg.x_pos = [0.0, 
                robotReach * np.cos(np.pi/6.0),
                robotReach * np.cos(np.pi/2.0),
                robotReach * np.cos(5.0*np.pi/6.0),
                robotReach * np.cos(7.0*np.pi/6.0),
                robotReach * np.cos(3.0*np.pi/2.0),
                robotReach * np.cos(11.0*np.pi/6.0)
    ]
    input_msg.y_pos = [0.0, 
                robotReach * np.sin(np.pi/6.0),
                robotReach * np.sin(np.pi/2.0),
                robotReach * np.sin(5.0*np.pi/6.0),
                robotReach * np.sin(7.0*np.pi/6.0),
                robotReach * np.sin(3.0*np.pi/2.0),
                robotReach * np.sin(11.0*np.pi/6.0)
    ]
    input_msg.z_pos = [ 0.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0 ]
    
    test_node.publisher.publish(input_msg)
    test_node.input_sent = True

    # Wait for the kin_node to process the input and publish the output
    rclpy.spin_once(test_node, timeout_sec=1.5)

    # Check if the kin_node published the expected output message
    assert test_node.output_received == True, 'OUTPUT NOT RECEIVED'

    # Check the angles obtained from the kin_node. Because of the star shape, everything should be 90 90 45
    for index in range(1, 7):
        assert round(test_node.out.shoulder_angle[index], 0) == 90.0
        assert round(test_node.out.hip_angle[index], 0) == 90.0
        assert round(test_node.out.knee_angle[index], 0) == 45.0

    # Cleanup
    test_node.destroy_node()
    kin_process.kill()
    rclpy.shutdown()
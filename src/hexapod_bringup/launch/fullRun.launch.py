from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    servo_node = Node(
        package = "leg_controller",
        executable = "servo_node"
    )
    kin_node = Node(
        package = "leg_controller",
        executable = "kin_node"
    )
    ctrl_node = Node(
        package = "leg_controller",
        executable = "ctrl_node"
    )

    ld.add_action(servo_node)
    ld.add_action(kin_node)
    #ld.add_action(ctrl_node)
    

    return ld
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Robot constants
    COXA_LEN = 71.5
    FEMUR_LEN = 100.02
    FOOT_HEIGHT = 0.0
    TIBIA_LEN = 150 + FOOT_HEIGHT
    GAIT_ALTITUDE = 90.0
    BASE_WIDTH = 65.0
    GAIT_WIDTH = 300.0
    
    ld = LaunchDescription()

    servo_node = Node(
        package = "leg_controller",
        executable = "servo_node",
    )
    kin_node = Node(
        package = "leg_controller",
        executable = "kin_node",
        parameters = [
            {"coxa_len": COXA_LEN},
            {"femur_len": FEMUR_LEN},
            {"tibia_len": TIBIA_LEN},
            {"gait_altitude": GAIT_ALTITUDE},
            {"base_width": BASE_WIDTH}
        ]
    )
    ctrl_node = Node(
        package = "leg_controller",
        executable = "ctrl_node",
        parameters = [
            {"coxa_len": COXA_LEN},
            {"femur_len": FEMUR_LEN},
            {"tibia_len": TIBIA_LEN},
            {"gait_altitude": GAIT_ALTITUDE},
            {"base_width": BASE_WIDTH},
            {"gait_width": GAIT_WIDTH}
        ]
    )

    ld.add_action(servo_node)
    ld.add_action(kin_node)
    #ld.add_action(ctrl_node)
    

    return ld
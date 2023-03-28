# Hexapod

## Project Description

As part of the Robotics and Intelligent Devices course, in the final year a final year project has to be undertaken by each student.

For my final year project I chose to fully research, design and develeop a hexapod robot with the appropriate control strategy to traverse highly diffused terrain. I also wanted to apply best practices in order to maximize the quality of the robot while minimizing the price, as I would like to make it a good research and hobby platform for people to take and work with.

## OPEN-SOURCE

Please note that the project is open-source, so feel free to fork it. I must mention that if you plan to work on the robot, please read the issues first as they might indicate a future feature I am working on that you might find useful. The project is work-in-progress, so it is worth checking it often for updates!

## CAD and 3D printing

The version uploaded in the CAD folder is currently printed in PLA. It should be noted that this was a fast placeholder for the robot and an updated version will come soon with leg end-effector switches that enables it to traverse the highly diffused terrain. An image of the CAD assembly ca be seen below

![CAD of Robot](CAD/side_view.png?raw=true "Hexapod Assembly")
![CAD of Robot](CAD/front_view.png?raw=true "Hexapod Assembly")

## Codebase

The hexapod's codebase is designed to work with Ubuntu 22.04 Server and ROS2 Humble. I will not work on backwards compatibility with previous versions of ROS/Ubuntu. When cloning the repository, please ensure you properly install ROS2 Humble (do the talker listener demo to ensure so). Once cloned, go to the parent directory and build it.

```
colcon build
```

At the time of writing, there are 2 fully integrated run modes:

1. Keyboard TeleOp

    For this mode you will need to terminals. One for the launch file launching all the necessary nodes to run the robot and one for the keyboard TeleOp.

    Terminal 1:
    ```
    ros2 launch hexapod_bringup stdRun.launch.py
    ```
    Terminal 2:
    ```
    ros2 launch hexapod_controller teleop_keyboard_node
    ```

    On the second terminal a list of the controls will appear and you'll be able to control the robot similarly to the turtlesim teleop example in ROS2

2. Gamepad TeleOp (PS style controller)

    Small note: Currently working on introducing XBox style controller support.

    For this mode a single terminal is needed which initializes the robot and you can control the robot through a controller connected via USB or bluetooth. I recommend using a wireless controller with an USB dongle. The code for running in this mode:

    ```
    ros2 launch hexapod_bringup gamepadMode.launch.py
    ```

## Electronics

I am currently working on finalizing the electronics design to the final version. Will include a schematic and Bill-Of-Materials as soon as available.

The robot runs on a Raspberry Pi 4B with 8Gb of RAM. I experimented with the 1Gb variant and it is barely enough, leaving very little headroom. For this reason I recommend at least 2Gb of RAM.

## When to expect a well rounded version?

By the start of June 2023 a full version with everything needed to build and use the robot will be available.
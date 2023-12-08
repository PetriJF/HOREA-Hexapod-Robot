# ROS2 Hexapod (IMPORTANT: CODE REFACTORING ONGOING!!)
![HOREA](CAD/H.O.R.E.A.1.png?raw=true "HOREA")

## IMPORTANT: Code refactoring in progress

Hello everyone! Just letting you know that the code is currently being refactored for the robot. The Hardware design of the robot was simplified, making it easier to work with and even cheaper, and those changes need to be reflected into the walking model as well.
A quick list of some of the features in progress:
- Dynamic Gait Switching
- Leg Fault handling
- Gazebo integration
- Vision/LiDAR support
- Better step feedback
- Better femur design
- Jumping
- Enabling the hexapod to walk on 4 or 5 legs
- Enabling the hexapod to walk in a crawl state (reducing the profile widthwise for fitting into thigh spots)
- A more efficient node communication method for the data heavy transfers

WHY NOT WORKING HERE?
Because I am using this version of the codebase for demos and project sharing. I don't want to add something that might be unstable or buggy here

WHEN WILL THE CHANGES BE UPLOADED?
My plan for it is to have the new hardware design uploaded by the end of December with the code changes required to run the software on it. All the other features should be online by late January 2023 when the private branch will be fully integrated with main.

SHOULD I BUILD THE CURRENT VERSION OF THE ROBOT?
Personally, I would say no, as the new version fixes a lot of the problems this iteration has. The new version will also be cheaper and easier to assemble and service! If you can't wait, email me at "jamespetri28@gmail.com" and we can work together for a quicker solution!

Thank you everyone,
James

## Project Description

As part of the Robotics and Intelligent Devices degree, in the final year a project has to be undertaken by each student.

For my final year project I chose to fully research, design and develeop a hexapod robot with the appropriate control strategy to traverse diffused terrain. I also wanted to apply best practices in order to maximize the quality of the robot while minimizing the price, as I would like to make it a good research and hobby platform for people to take and work with.

The Hexapod, named H.O.R.E.A., works on a raspberry Pi running ROS2 humble. It comes equipped with an IMU, for tilt correction, and a switch in each tibia, for leg feedback. With those two perception methods, the hexapod is fully prepared to undertake light to medium off-road challanges (Relative to the robot's size). Some metrics are available at the end.

In the future, the platform might add vision and a more robust control design in order to improve its capabilities even further.

## OPEN-SOURCE

Please note that the project is open-source, so feel free to fork it. I must mention that if you plan to work on the robot, please read the issues first as they might indicate a future feature I am working on that you might find useful. The project is work-in-progress, so it is worth checking it often for updates!

## CAD and 3D printing

The version current version of the hexapod is fully 3D printed in ABS, except for the TibiaTips, which are printed in TPU in order to provide more grip. Some images of the CAD assembly can be seen below

![CAD of Robot](CAD/side_view.png?raw=true "Hexapod Assembly")
![CAD of Robot](CAD/front_view.png?raw=true "Hexapod Assembly")
![CAD of Robot](CAD/top_view.png?raw=true "Hexapod Assembly")

## Codebase

The hexapod's codebase is designed to work with Ubuntu 22.04 Server and ROS2 Humble. I will not work on backwards compatibility with previous versions of ROS/Ubuntu. When cloning the repository, please ensure you properly install ROS2 Humble (do the talker listener demo to ensure so). Once cloned, go to the parent directory and build it.

```
colcon build
```

At the time of writing, there are 3 fully integrated run modes separated into launch files under hexapod_bringup. One of them is a keyboard controller, while the other two are gamepad controllers(one for generic style, and one for XBox style). The run procedure for the keyboard and XBox controllers can be seen below:

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

    For this mode a single terminal is needed which initializes the robot and you can control the robot through a controller connected via USB or bluetooth. I recommend using a wireless controller with an USB dongle. The code for running in this mode:

    ```
    ros2 launch hexapod_bringup xBoxGamepadMode.launch.py
    ```

## Bill Of Materials

A full bill of materials can be found under the CAD directory. Look for "Full BOM.xlsx"

## When to expect a well rounded version?

By the start of June 2023 a full version with everything needed to build and use the robot will be released. The robot is currently in working order (08/05/2023), but some of the off-road features are on development branches. 

## Metrics

| Metric | Value |
| :---: | :---:|
|Weight | 2.68 Kg |
|Payload | 2.4 Kg |
|Maximum leg radius | 87 cm |
|Leg Reach | 37 cm |
|Robot Height | 17.6 cm (23.6 cm with  gamepad antenna) |
|Maximum dynamic incline (rough surface) | 23.5° |
|Maximum static incline (rough surface) | 35.3° |
|Maximum dynamic incline (smooth surface) | 10.3° |
|Maximum static incline (smooth surface) | 19.6° |
|Speed (Tripod gait) | 0.253 m/s |


## More pictures of H.O.R.E.A.

![HOREA](CAD/1.png?raw=true "HOREA")
![HOREA](CAD/2.png?raw=true "HOREA")
![HOREA](CAD/3.png?raw=true "HOREA")
![HOREA](CAD/5.png?raw=true "HOREA")
![HOREA](CAD/4.png?raw=true "HOREA")

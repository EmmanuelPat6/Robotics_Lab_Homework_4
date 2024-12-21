# HOMEWORK_4 PATELLARO EMMANUEL P38000239 #
# 🤖📱 Control a Mobile Robot to Follow a Trajectory 🖥️🎯 #
This README file will show the instructions on how to build and run the Homework_4 Project 

## Features 🪐 ##
- Computer Vision 💻👁️
- Gazebo and Mobile Robot 🌎🚗
- ArUco Tag 🏷️🧩
- Autonomous Navigation Task 🚀🧭
- Follow Waypoints 📍➡️📍➡️
- Environment Mapping 🗺️🔍
- Vision-Based Navigation 📷👀
- TF Publishing 📡🔃

## Available Directory in this Repository 📂 ##
- rl_fra2mo_description
- ros2_vision
- bag

## Getting Started ⏯️
1. Follow the guide to install ROS2 in Docker [here](https://github.com/RoboticsLab2024/ros2_docker_scripts.git)
2. Clone this repo in your `src` folder inside the `ros2_ws`
    ```shell
    cd src
    git clone https://github.com/EmmanuelPat6/Homework_3.git
    ```
3. Build the packages and make them visible to your workspace
    ```shell
    cd ..
    colcon build
    source install/setup.bash
    ```

## Implementation 💻
### Follow 4 given Goals

1. 🤖🤖 An instruction to spawn the robot in Gazebo
    ```shell
    ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
    ```
⚠️⚠️⚠️ It is **NECESSARY** to press the play button in the bottom left corner to run the next instructions ⚠️⚠️⚠️

2. 🚀🧭 An istruction to launch `SLAM`, `Exploration` and `Nav2`
    ```shell
    ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
    ```
    
3. 🔧⚙️ An istruction to run the `async_slam_toolbox_node`
    ```shell
    ros2 launch rl_fra2mo_description fra2mo_slam.launch.py
    ```
    
### Vision-Based Control with Velocity Commands 🏎️📷

1. 🤖🤖 An instruction to spawn the robot in Gazebo inside the world containing the **ArUco Tag 201** with a **Velocity Controller**
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
    ```
    `command_interface:="velocity"` and `robot_controller:="velocity_controller"` to spawn the robot with a **Velocity Interfare** and a **Velocity Controller**, `use_sim:=true` to spawn the robot in Gazebo and `use_vision:=true` to spawn the robot with a **Camera Sensor**

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. ⚠️⚠️⚠️

2. 📐📏 An istruction to allow the **ArUco Tag 201 Detection**  
    ```shell
    ros2 launch aruco_ros single.launch.py 
    ```
   
3. 🗺️📸 An istruction to see the environment with the **ArUco Detection** through the **Camera Sensor**
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
    selecting the topic `/aruco_single/result`

4. 🚀📍An instruction to do the **Positioning Task** in order to align the **Camera** to the **ArUco Marker** with a desired **Position and Orientation Offsets**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_control 
    ```
5. 👀🎯After that the **Positioning** is completed (wait the message `Positioning Task Executed Successfully ...`) it is possible to run in this last terminal, after pressing `ctrl+C`, the final instruction
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_control --ros-args -p task:=look-at-point
    ```
   which performs a **Look-at-Point Task** using a desired **Control Law** specified in the code and in the report. Now it is possible to move the ArUco Tag with the realtive interface and the center of the **Camera Sensor** will align with      the center of the Tag.

 ### Vision-Based Control with Effort Commands 🦾📷
For this another file called `ros2_kdl_vision_effort_control.cpp` has been implemented.

1. 🤖🤖 An instruction to spawn the robot in Gazebo inside the world containing the **ArUco Tag 201** with a **Effort Controller**
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:=true use_vision:=true
    ```
    `command_interface:="velocity"` and `robot_controller:="velocity_controller"` to spawn the robot with a **Velocity Interfare** and a **Velocity Controller**, `use_sim:=true` to spawn the robot in Gazebo and `use_vision:=true` to spawn the robot with a **Camera Sensor**

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. ⚠️⚠️⚠️

2. 📐📏 An istruction to allow the **ArUco Tag 201 Detection**  
    ```shell
    ros2 launch aruco_ros single.launch.py 
    ```
   
3. 🗺️📸 An istruction to see the environment with the **ArUco Detection** through the **Camera Sensor**
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
    selecting the topic `/aruco_single/result`

4. 🚀📍An instruction to do the **Positioning Task** in order to align the **Camera** to the **ArUco Marker** with a desired **Position and Orientation Offsets**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_effort_control --ros-args -p task:=positioning
    ```
5. 👀📏An instruction to do the **Look-at-Point Task** with a **Linear Trajectory**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_effort_control --ros-args -p task:=look-at-point
    ```
   which, using a new **Orientation Error** given by `sd-s`, implement the **Linear Trajectory** implemented in the previous Homework but looking, during it, the **ArUco Tag**. It is advisable not to run this instruction after the               positioning but to re-execute the initial instructions and respawn the robot in its initial position.

To implement these last two points with the **Operational Space Inverse Dynamics Control** it is sufficient to add at teh end of each instruction `-p cmd_interface:=cart_effort`

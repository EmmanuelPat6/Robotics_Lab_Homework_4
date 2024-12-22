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
    git clone https://github.com/EmmanuelPat6/Homework_4.git
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
    
4. 🌐🔭 An instruction to start `Rviz2` with the `explore.rviz` configuration
   ```shell
   ros2 launch rl_fra2mo_description display_fra2mo.launch.py
   ```
   
5. ➡️📍➡️📍➡️📍➡️📍 An instruction to follow the 4 given Waypoints
   ```shell
   ros2 run rl_fra2mo_description follow_waypoints.py 
   ```
    ⚠️⚠️⚠️ By default the `Initial Position` of the Robot in Gazebo is the `Origin`. It is possible, according to the Homework specification, to change it in the file `gazebo_fra2mo.launch.py` at `line 60` in `position = [-3.0, 3.5, 0.100, -1.57]`
    (by default it is `position = [0.0, 0.0, 0.100, 0.0]`). In this case, this `5.` instruction must be different⚠️⚠️⚠️
   ```shell
   ros2 run rl_fra2mo_description follow_waypoints_no_origin.py 
   ```
    
### Map the Environment 🔃🛰️
The instructions are mostly the same as before. In this case, only the `Initial Position` in the `Origin` will be considered (so, if you change in the previous point the position in `gazebo_fra2mo.launch.py`, let's modify it again in `position = [0.0, 0.0, 0.100, 0.0]` by default)

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
    
4. 🌐🔭 An instruction to start `Rviz2` with the `explore.rviz` configuration
   ```shell
   ros2 launch rl_fra2mo_description display_fra2mo.launch.py
   ```
   
5. 🗺️🌎 An instruction to follow 5 Waypoints in order to Map the entire Environment
   ```shell
   ros2 run rl_fra2mo_description follow_more_waypoints.py 
   ```

The `Map` obtained by this implementation is shown not only in the Report but there is also the file `map.pgm` in this Repository.


 ### Vision-Based Navigation 📷🛤️

1. 🤖🤖 An instruction to spawn the robot in Gazebo
    ```shell
    ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
    ```
⚠️⚠️⚠️ It is **NECESSARY** to press the play button in the bottom left corner to run the next instructions ⚠️⚠️⚠️

2. 🚀🧭 An istruction to launch both `Navigation` and `aruco_ros` node using the Robot `Camera` added
    ```shell
    ros2 launch rl_fra2mo_description fra2mo_aruco.launch.py 
    ```
    
3. 📷🎥 An istruction to show what the fra2mo Robot see and the detection of the ArUco Tag
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
   and select the topic `/aruco_single/result`

4. ⛵🤙 An instruction to implement a `2D Navigation Task` following this logic
   - Send the robot in the proximiti of `Obstacle 9` ➡️🛑
   - Make the Robot look for the `ArUco Marker`. Once detected, its `Pose` is retrieved with respect the `Map Frame` 📷🔍
   - Return the Robot to the `Initial Position` 🔙🏁
     
   ```shell
   ros2 run rl_fra2mo_description follow_aruco_waypoints.py 
   ```
   ⚠️⚠️⚠️ As before,y default the `Initial Position` of the Robot in Gazebo is the `Origin`. It is possible, according to the Homework specification, to change it in the file `gazebo_fra2mo.launch.py` at `line 60` in `position = [-3.0, 3.5, 0.100, -1.57]`
    (by default it is `position = [0.0, 0.0, 0.100, 0.0]`). In this case, this `4.` instruction must be different⚠️⚠️⚠️
   ```shell
   ros2 run rl_fra2mo_description follow_aruco_waypoints_no_origin.py 
   ```
   This is **NECESSARY** because, in the second case, the `Map Frame` is different and some additional steps are required to publish the `ArUco Pose` while maintaining the reference of `Gazebo Map Frame`

 ### ArUco TF Publishing 📸🔄
 5. In addition to the instructions of the previous point, it is possible to add an additional instruction that allows me to print the `ArUco Pose` on the screen as a `STATIC TF` once detected. To do this run:
    ```shell
    ros2 topic echo tf_static
    ```


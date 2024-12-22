#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math


goals_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "goals.yaml")
with open(goals_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)


def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        translation = {"x": -3, "y": 3.5}
        rotation = -math.pi / 2

        map_position = transform["position"]
        map_orientation = transform["orientation"]

        x_dash = map_position["x"] - translation["x"]
        y_dash = map_position["y"] - translation["y"]

        #Inverse Rotation
        new_x = math.cos(-rotation) * x_dash - math.sin(-rotation) * y_dash
        new_y = math.sin(-rotation) * x_dash + math.cos(-rotation) * y_dash

        rot_q = {
            "x": 0,
            "y": 0,
            "z": math.sin(-rotation / 2),
            "w": math.cos(-rotation / 2)
        }

        new_q = {
            "x": rot_q["w"] * map_orientation["x"] + rot_q["x"] * map_orientation["w"] + rot_q["y"] * map_orientation["z"] - rot_q["z"] * map_orientation["y"],
            "y": rot_q["w"] * map_orientation["y"] - rot_q["x"] * map_orientation["z"] + rot_q["y"] * map_orientation["w"] + rot_q["z"] * map_orientation["x"],
            "z": rot_q["w"] * map_orientation["z"] + rot_q["x"] * map_orientation["y"] - rot_q["y"] * map_orientation["x"] + rot_q["z"] * map_orientation["w"],
            "w": rot_q["w"] * map_orientation["w"] - rot_q["x"] * map_orientation["x"] - rot_q["y"] * map_orientation["y"] - rot_q["z"] * map_orientation["z"]
        }

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = new_x
        pose.pose.position.y = new_y
        pose.pose.position.z = map_position["z"]
        pose.pose.orientation.x = new_q["x"]
        pose.pose.orientation.y = new_q["y"]
        pose.pose.orientation.z = new_q["z"]
        pose.pose.orientation.w = new_q["w"]
        return pose

    goals = list(map(create_pose, yaml_content["waypoints"]))

    # Order: Goal 3 → Goal 4 → Goal 2 → Goal 1
    reordered_goal_indices = [2, 3, 1, 0]  # Python indexing starts at 0
    goal_poses = [goals[i] for i in reordered_goal_indices]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
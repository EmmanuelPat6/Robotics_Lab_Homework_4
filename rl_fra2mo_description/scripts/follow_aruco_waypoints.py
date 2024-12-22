#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import tf2_geometry_msgs
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
import yaml, os, math
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros import TransformException
from rclpy.duration import Duration
from rclpy.time import Time
import time

goals_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "aruco_goals.yaml")
with open(goals_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.aruco_sub = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            10
            )
        
        self.aruco_pose = None

        self.offset_x = 0.0
        self.offset_y = 0.0

        #self.offset_x = -3.0
        #self.offset_y = 3.5

    def aruco_navigation(self):

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active(localizer="smoother_server")
        
        # Near Obstacle 9
        self.go_near_obstacle()

        # ArUco Detection
        self.aruco_detection()

        # Back
        self.return_to_initial_position()


    def go_near_obstacle(self):

        only_one_print = False
        goals = list(map(self.create_pose, yaml_content["waypoints"]))

        # Near Obstacle 9 
        reordered_goal_indices = [0]
        goal_poses = [goals[i] for i in reordered_goal_indices]

        self.navigator.followWaypoints(goal_poses)

        # Wait Until Navigation is Completed
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and not only_one_print:
                print(f'Go Near Obstacle 9')
                only_one_print = True


    def aruco_detection(self):

        # Wait until Detection
        while not self.aruco_pose:
            rclpy.spin_once(self)

        self.get_logger().info(f"ArUco Tag Position in Map Frame: "
                               f"Position -> [x: {self.aruco_pose.pose.position.x}, "
                               f"y: {self.aruco_pose.pose.position.y}, z: {self.aruco_pose.pose.position.z}], "
                               f"Orientation -> [x: {self.aruco_pose.pose.orientation.x}, "
                               f"y: {self.aruco_pose.pose.orientation.y}, "
                               f"z: {self.aruco_pose.pose.orientation.z}, "
                               f"w: {self.aruco_pose.pose.orientation.w}]")


    def aruco_callback(self, msg):

        try:
            # Camera Frame
            source_frame = msg.header.frame_id

            # Message Time
            timestamp = msg.header.stamp

            # Transformation from Camera Frame and Map Frame
            transform = self.tf_buffer.lookup_transform(
                "map", source_frame, timestamp, timeout=Duration(seconds=0.0))

            # ArUco Pose with respect Camera Frame
            aruco_position = msg.pose.position
            aruco_orientation = msg.pose.orientation

            # URDF Offsets
            base_offset = [0.0825, 0.0, 0.04]
            camera_offset = [0.1, 0.0, 0.072]

            # ArUco Pose with respect Map Frame
            aruco_map_pose = PoseStamped()
            aruco_map_pose.header.frame_id = "map"
            aruco_map_pose.header.stamp = self.get_clock().now().to_msg()
            aruco_map_pose.pose.position.x = transform.transform.translation.x - aruco_position.x - camera_offset[0] + base_offset[0]
            aruco_map_pose.pose.position.y = transform.transform.translation.y - aruco_position.z + camera_offset[1] + base_offset[1]
            aruco_map_pose.pose.position.z = transform.transform.translation.z - aruco_position.y + camera_offset[2] + base_offset[2]
            aruco_map_pose.pose.orientation = aruco_orientation

            # ArUco Pose for function aruco_detection
            self.aruco_pose = aruco_map_pose


            # ArUco Pose as TF
            t = TransformStamped()

            # Timestamp
            t.header.stamp = self.get_clock().now().to_msg()

            t.header.frame_id = "map"
            t.child_frame_id = "aruco_pose_tf"

            # TF Position
            t.transform.translation.x = aruco_map_pose.pose.position.x
            t.transform.translation.y = aruco_map_pose.pose.position.y
            t.transform.translation.z = aruco_map_pose.pose.position.z

            # TF Orientation
            t.transform.rotation.x = aruco_map_pose.pose.orientation.x
            t.transform.rotation.y = aruco_map_pose.pose.orientation.y
            t.transform.rotation.z = aruco_map_pose.pose.orientation.z
            t.transform.rotation.w = aruco_map_pose.pose.orientation.w

            # Send TF
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().info("\nArUco TF Published Successfully\n")

        except TransformException as e:
            self.get_logger().error(f"Error: {e}")



    def return_to_initial_position(self):

        only_one_print = False
        goals = list(map(self.create_pose, yaml_content["waypoints"]))

        # Initial Position
        reordered_goal_indices = [1]
        goal_poses = [goals[i] for i in reordered_goal_indices]

        self.navigator.followWaypoints(goal_poses)

        # Wait Until Navigation is Cpmpleted
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and not only_one_print:
                print(f'Back To Initial Position')
                only_one_print = True



    def create_pose(self, transform):

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose



def main():
    rclpy.init()
    node = ArucoNode()
    node.aruco_navigation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
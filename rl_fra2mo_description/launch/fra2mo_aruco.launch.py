from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    aruco_dir = FindPackageShare('aruco_ros')

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_explore.launch.py'])))


    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_dir, 'launch', 'single.launch.py'])))



    return LaunchDescription(
        [
            explore_launch,
            aruco_launch,
        ]
    )

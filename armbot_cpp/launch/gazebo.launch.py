from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    armbot_cpp = get_package_share_directory("armbot_cpp")
    armbot_cpp_prefix = get_package_prefix("armbot_cpp")

    model_path = os.path.join(armbot_cpp, "models")
    model_path += pathsep + os.path.join(armbot_cpp_prefix, "share")

    # Corrected: Set environment variable
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("armbot_cpp"), "urdf", "bot.urdf.xacro"),
        description="Absolute path to the robot urdf file"
    )

    # Robot description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Gazebo server and client launch descriptions
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        )
    )
    
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "armbot_cpp", "-topic", "robot_description"],
        output="screen"
    )

    # Return the LaunchDescription with all nodes and launch actions
    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,  # Included the missing node
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="gripper_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_interface_file",
            default_value="gripper_hardware_interface.yaml",
            description="YAML file with the hardware interface configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ethernet_interface",
            default_value="enp3s0",
            description="EtherCAT network interface name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_slave_id",
            default_value="1",
            description="Gripper Modbus slave ID.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_speed",
            default_value="80.0",
            description="Gripper movement speed.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_force",
            default_value="50.0",
            description="Gripper force setting.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="gripper_controller",
            description="Robot controller to start.",
        )
    )

    # Initialize Arguments
    controllers_file = LaunchConfiguration("controllers_file")
    hardware_interface_file = LaunchConfiguration("hardware_interface_file")
    ethernet_interface = LaunchConfiguration("ethernet_interface")
    gripper_slave_id = LaunchConfiguration("gripper_slave_id")
    gripper_speed = LaunchConfiguration("gripper_speed")
    gripper_force = LaunchConfiguration("gripper_force")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("crt_ctag2f90c_gripper_visualization"), "urdf", "crt_ctag2f90c_model.xacro"]
            ),
            " ",
            "ethernet_interface:=",
            ethernet_interface,
            " ",
            "gripper_slave_id:=",
            gripper_slave_id,
            " ",
            "gripper_speed:=",
            gripper_speed,
            " ",
            "gripper_force:=",
            gripper_force,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get package share directory
    pkg_share = FindPackageShare("gripper_hardware_interface")
    
    # Get controller configuration file
    controllers_file_path = PathJoinSubstitution(
        [pkg_share, "config", controllers_file]
    )
    
    # Get hardware interface configuration file
    hardware_interface_file_path = PathJoinSubstitution(
        [pkg_share, "config", hardware_interface_file]
    )

    # Controller configuration
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file_path,
            hardware_interface_file_path,
        ],
        output="both",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Gripper controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    # Delay start of gripper controller after joint_state_broadcaster
    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

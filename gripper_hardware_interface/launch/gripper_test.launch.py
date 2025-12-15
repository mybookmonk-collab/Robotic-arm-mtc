#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
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

    # Initialize Arguments
    ethernet_interface = LaunchConfiguration("ethernet_interface")
    gripper_slave_id = LaunchConfiguration("gripper_slave_id")

    # Get package share directory
    pkg_share = FindPackageShare("gripper_hardware_interface")
    
    # Get configuration files
    hardware_interface_file = PathJoinSubstitution(
        [pkg_share, "config", "gripper_hardware_interface.yaml"]
    )
    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "gripper_controllers.yaml"]
    )

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
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            hardware_interface_file,
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
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
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
        # Test node removed
    ]

    return LaunchDescription(declared_arguments + nodes)

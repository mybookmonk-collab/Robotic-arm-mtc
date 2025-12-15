#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
import yaml


def generate_launch_description():

    # Gazebo world file
    elfin3_ros2_gazebo = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'worlds',
        'elfin3.world')

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                'gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ]),
        launch_arguments={'world': elfin3_ros2_gazebo}.items(),
    )

    # Robot description (URDF xacro processed)
    elfin3_description_path = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'))
    xacro_file = os.path.join(elfin3_description_path,
                              'urdf',
                              'elfin3.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=true',
         ])

    robot_description = {'robot_description': robot_description_config}

    # ros2_control_node 启动，加载robot_description和控制器配置参数
    # 注意要把你的controller配置yaml路径改成实际路径
    controller_yaml = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'config',
        'elfin_arm_controller.yaml'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml],
        output='screen'
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'elfin3',
                   "-x", "0.0", "-y", "0.0", "-z", "0.1"],
        output='screen'
    )

    # Load controllers commands
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'elfin_arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # 事件链：spawn_entity退出后加载joint_state_broadcaster，joint_state_broadcaster退出后加载其他控制器
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller,
                     load_gripper_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        ros2_control_node,          # 新增启动ros2_control_node
        spawn_entity,
        node_robot_state_publisher,
        close_evt1,
        close_evt2,
    ])



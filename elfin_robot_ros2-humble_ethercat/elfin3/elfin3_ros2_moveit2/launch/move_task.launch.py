#!/usr/bin/python3

# elfin3.launch.py:
# Launch file for the elfin3 Robot GAZEBO + MoveIt!2 SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable,LaunchConfiguration
import xacro
import yaml

# LOAD FILE:


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:

        return None
# LOAD YAML:


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    # 声明抓取点参数
    declare_pose_x = DeclareLaunchArgument("target_x", default_value="0.4")
    declare_pose_y = DeclareLaunchArgument("target_y", default_value="0.5")
    declare_pose_z = DeclareLaunchArgument("target_z", default_value="0.3")
    declare_pose_qx = DeclareLaunchArgument("target_qx", default_value="-0.5")
    declare_pose_qy = DeclareLaunchArgument("target_qy", default_value="-0.5")
    declare_pose_qz = DeclareLaunchArgument("target_qz", default_value="-0.5")
    declare_pose_qw = DeclareLaunchArgument("target_qw", default_value="0.5")

    # ***** ROBOT DESCRIPTION ***** #
    # elfin3 Description file package:
    elfin3_description_path = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'))
    # elfin3 ROBOT urdf file path:
    xacro_file = os.path.join(elfin3_description_path,
                              'urdf',
                              'elfin3.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for elfin3:
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=false',])

    robot_description = {'robot_description': robot_description_config}

    load_controllers = []
    for controller in [
        "elfin_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(
                    controller)],
                shell=True,
                output="screen",
            )
        ]
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("elfin3_ros2_gazebo"),
            "urdf",
            "elfin3.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    # Robot description, SRDF:
    robot_description_semantic_config = load_file(
        "elfin3_ros2_moveit2", "config/elfin3.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "elfin3_ros2_moveit2", "config/kinematics.yaml"
    )

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "elfin3_ros2_moveit2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "elfin3_ros2_moveit2", "config/elfin_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_demo = Node(
        package="mtc_tutorial",
        executable="move_task",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
            {"target_x": LaunchConfiguration("target_x")},
            {"target_y": LaunchConfiguration("target_y")},
            {"target_z": LaunchConfiguration("target_z")},
            {"target_qx": LaunchConfiguration("target_qx")},
            {"target_qy": LaunchConfiguration("target_qy")},
            {"target_qz": LaunchConfiguration("target_qz")},
            {"target_qw": LaunchConfiguration("target_qw")}
        ],
    )

    return LaunchDescription(
        [
            move_group_demo
        ]
    )


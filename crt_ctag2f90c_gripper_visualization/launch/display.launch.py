import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the URDF and RViz configuration
    urdf_file = os.path.join(
        get_package_share_directory('crt_ctag2f90c_gripper_visualization'),
        'urdf', 'crt_ctag2f90c.urdf'
    )
    rviz_file = os.path.join(
        get_package_share_directory('crt_ctag2f90c_gripper_visualization'),
        'urdf.rviz'
    )

    # Print paths for debugging
    print("URDF File Path:", urdf_file)
    print("RViz File Path:", rviz_file)

    # Read the URDF file content into a string
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Declare model argument (optional, you can use it to pass different URDFs if needed)
        DeclareLaunchArgument(
            'model', default_value='', description='Model argument'
        ),

        # Robot description parameter (loading URDF content)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_file]
        ),
    ])


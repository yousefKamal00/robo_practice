from os import path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=path.join(
            get_package_share_directory('control_test'),
            'urdf',
            'description.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    sim_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    # rviz_config_file = LaunchConfiguration('rviz_config', default='$(find control_test)/config/rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', '$(find control_test)/config/rviz.rviz'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
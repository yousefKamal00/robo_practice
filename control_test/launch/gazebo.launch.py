from os import path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    gazebo_share_dir = get_package_share_directory("gazebo_ros")

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

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(gazebo_share_dir, "launch", "gzserver.launch.py")
        )
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(gazebo_share_dir, "launch", "gzclient.launch.py")
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_share_dir,
                "launch",
                "gazebo.launch.py"
            ])
        ])
        # launch_arguments={"world": "empty.world"}.items()
    )

    gz_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "test_robot",
                   "-topic", "robot_description"]
    )

    state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["broadcaster"],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],

    )

    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller"],
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
    )
    
    # rviz_config_file = LaunchConfiguration('rviz_config', default='$(find control_test)/config/rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', '$(find control_test)/config/rviz.rviz'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    delay_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=state_broadcaster_spawner_node,
            on_exit=[controller_spawner_node],
        )
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[path.join(get_package_share_directory("control_test"), "config", "joy_config.yaml")]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        output="screen",
        parameters=[path.join(get_package_share_directory("control_test"), "config", "joy_teleop.yaml")]
    )

    return LaunchDescription([
        model_arg,
        sim_arg,
        robot_state_publisher_node,
        # rviz_node,
        # gz_server,
        # gz_client,
        gazebo,
        gz_spawn_entity_node,
        state_broadcaster_spawner_node,
        # controller_spawner_node,
        delay_diff_drive_controller_spawner,
        joy_node,
        joy_teleop
    ])
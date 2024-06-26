#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    omo_dir = get_package_share_directory('omo_r1mini_gazebo')
    urdf_dir = get_package_share_directory('omo_r1mini_description')
    my_custom_pkg = get_package_share_directory('omo_r1mini_custom')
    
    #cartographer
    cartographer_pkg = os.path.join(get_package_share_directory('omo_r1mini_cartographer'), 'launch')
    
    #rviz2
    #if you have your rviz file, replace it.
    # rviz2_p = os.path.join(
    #     my_custom_pkg,
    #     'rviz', '**.rviz')
    
    #world_map
    world = os.path.join(
        my_custom_pkg,
        'world', 'maze_Astar.world')

    ld = LaunchDescription()

    # Names and poses of the robots
    robot = {'name': '', 'x_pose': '0', 'y_pose': '0', 'z_pose': 0.01}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    urdf = os.path.join(
        urdf_dir, 'urdf', 'omo_r1mini' + '.urdf'
    )
    sdf = os.path.join(omo_dir,'models', 'omo_r1mini', 'model.sdf')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    # Declare the launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None

    # Spawn robot instances in gazebo
    namespace = [ '/' + robot['name'] ]

    # Create state publisher node for that instance
    robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                        'publish_frequency': 10.0}],
        remappings=remappings,
        arguments=[urdf],
    )

    # Create spawn call
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', sdf,
            '-entity', robot['name'],
            '-robot_namespace', namespace,
            '-x', robot['x_pose'], '-y', robot['y_pose'],
            '-z', '0.01', '-Y', '0.0',
            '-unpause',
        ],
        output='screen',
    )

    # cartographer
    # omo_cartographer = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([cartographer_pkg, '/cartographer.launch.py']),
    # )

    ros2_Astar = Node(
        package="omo_r1mini_custom",
        executable="ros2_Astar.py",
        arguments=[],
        output='screen',
    )

    if last_action is None:
        # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
        ld.add_action(robot_state_publisher)
        ld.add_action(spawn_robot)
        ld.add_action(ros2_Astar)
        # ld.add_action(omo_cartographer)

    else:
        # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
        # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
        spawn_turtlebot3_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[spawn_robot,
                        robot_state_publisher,
                        ],
            )
        )

        ld.add_action(spawn_turtlebot3_event)

    return ld

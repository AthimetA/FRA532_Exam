# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    description_package_name = 'pmzbbot_description'

    # # ***** GAZEBO ***** #   
    # # DECLARE Gazebo WORLD file:
    world_file = os.path.join(
        get_package_share_directory(description_package_name),
        'worlds',
        'office_pmzbbot.world')
    
    # # gzserver launch with world 
    gazebo_server_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # gzclient
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # ***** ROBOT DESCRIPTION ***** #
    # Add Robot description
    description_file_subpath = 'description/pmzbbot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(description_package_name),description_file_subpath) # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(     # Configure the node
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'pmzbbot',
                                   '-x', '0.0',
                                    '-y', '0.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        gazebo_server_with_world,
        gazebo_client,
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
    ])
import os
import pandas as pd

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def get_world_name(world):
    world_name = world.split(".")
    return world_name[0]


def generate_launch_description():
    # Launch Arguments:
    # Type of the UAV (fixed_wing, vtol):
    type_uav = "fixed_wing"
    # Launch airplane arguments:
    robot_name = "airplane_1"
    robot_scale = "0.7"
    camera_num = "1.0"
    # Launch arguments for the world:
    world = "ERAU.sdf"
    world_name = get_world_name(world)
    # Fog characteristis:
    fog_lifetime="10.0"
    fog_rate="1.0"
    enable_fog="false"

    # Package of the launch files:
    pkg_bringup = "uav_bringup"
    # Start the world:
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_bringup),
                'launch',
                'start_world.launch.py'
            ])
        ]),
        launch_arguments = {
            'world': PathJoinSubstitution([
                FindPackageShare(pkg_bringup),
                'worlds',
                world
            ]),
            'camera': TextSubstitution(text='true')
        }.items()
    )



    # Spawn the airplane;
    spawn_airplane = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_bringup),
                'launch',
                'uav_launcher.launch.py'
            ])
        ]),
        launch_arguments={
            'type_uav': type_uav,
            'robot_scale': robot_scale,
            'robot_name': robot_name,
            'camera_num': camera_num,
            'world_name': world_name,
            'fog_lifetime': fog_lifetime,
            'fog_rate': fog_rate,
            'enable_fog': enable_fog,
            'spawn_x':"0",
            'spawn_y':"0",
            'spawn_z':"50",
        }.items()
    )



    # Connect ros2 of the UAV with the Gazebo Sim:
    gz_camera_dridge = Node(
        package = 'ros_gz_bridge',
        executable ='parameter_bridge',
        arguments = [
            [f'/world/',world_name,'/model/',robot_name,'/link/base_link/sensor/follow_camera/image','@sensor_msgs/msg/Image[gz.msgs.Image'],
            [f'/world/', world_name, '/model/',robot_name,'/link/base_link/sensor/follow_camera/camera_info','@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings = [
            ([f'/world/',world_name,'/model/',robot_name,'/link/base_link/sensor/follow_camera/image'],[f'/',robot_name,'/camera/image_raw']),
            ([f'/world/', world_name, '/model/', robot_name, '/link/base_link/sensor/follow_camera/camera_info'],[f'/', robot_name, '/camera/camera_info']),
        ],
        output = 'screen'
    )


    return launch.LaunchDescription([
        start_world,
        spawn_airplane,
        gz_camera_dridge,
    ])
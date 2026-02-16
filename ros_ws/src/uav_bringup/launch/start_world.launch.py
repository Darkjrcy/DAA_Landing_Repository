
import os
from lxml import etree
from pathlib import Path


from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, AppendEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration



def launch_setup(context, *args, **kargs):

    # Declare the world path:
    world_path = LaunchConfiguration('world').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)

    # Add the model configutation to the agzebo sim path:
    pkg_bringup = get_package_share_directory('uav_bringup')
    models_path = os.path.join(pkg_bringup, 'models')   # e.g. …/plane_bringup/models

        




    # Add the airpalne movement plugin path:
    movement_plugins_lib = os.path.join(get_package_share_directory('movement_plugin'), '..', '..', 'lib')


    # Return the launhc command:
    return [
        # Add the resource file:
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            models_path 
        ),
        # Add the custom movement plugins path
        AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            f"{movement_plugins_lib}"  # To add more use :{},
        ),
        # Open the world:
        ExecuteProcess(
            cmd = [
                'gz','sim','-r',
                world_path,
                '--verbose'
            ],
            output = 'screen'
        ),
    ]





def generate_launch_description():
    plane_bringup_dir = get_package_share_directory('uav_bringup')
    default_world_path = os.path.join(plane_bringup_dir, 'worlds', 'ERAU.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Full path to the world model file to load'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Enable verbose Ignition output'
        ),
        OpaqueFunction(function=launch_setup)
    ])











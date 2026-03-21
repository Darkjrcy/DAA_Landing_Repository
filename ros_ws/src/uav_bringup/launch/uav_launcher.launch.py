import os
import yaml
import math

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction


# Function to define the Airplane characteristics:
def launch_setup(context,*args,**kwargs):

    # Input arguments:
    # Type of uav:
    type_UAV = LaunchConfiguration('type_uav').perform(context)
    # Name of the robot:
    robot_name = LaunchConfiguration('robot_name').perform(context)
    # Scale of the robot
    robot_scale = LaunchConfiguration('robot_scale').perform(context)
    # Number of cameras:
    camera_num = LaunchConfiguration('camera_num').perform(context)
    # Name of the world
    world_name = LaunchConfiguration('world_name').perform(context)
    # Position:
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_R = '0'
    spawn_P = '0'
    spawn_Y = '0'

    # Get the id number:
    uav_id = LaunchConfiguration('uav_id').perform(context)

    # In case of fog use: Remember Fog is still not well implemeneted in GZ sim
    fog_lifetime = LaunchConfiguration('fog_lifetime').perform(context)
    fog_rate = LaunchConfiguration('fog_rate').perform(context)
    enable_fog = LaunchConfiguration('enable_fog').perform(context)



    # Find the urdf directory
    # Package of the UAVs urdfs:
    package_description = "uav_description"
    # Base xavro file:
    xacro_file = f"{type_UAV}/main_{type_UAV}.xacro"
    # Define the xacro file:
    robot_xacro_path = os.path.join(get_package_share_directory(package_description),"urdf",xacro_file)


    # In case teh system the UAV type is a vtol chaneg to use matlab characteristics:
    # Get the config file:
    if type_UAV == "vtol":
        yaml_name = f"vtol_params_{uav_id}.yaml"
        config_path = os.path.join(get_package_share_directory("uav_bringup"), "config", yaml_name)

        # Open the file:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                params = data['/**']['ros__parameters']
                
                # Overwrite the spawn coordinates with MATLAB values
                spawn_x = str(params['east0'])
                spawn_y = str(params['north0'])
                spawn_z = str(params['alt0'])
                
                # Get matlab rotation
                matlab_heading = params['heading'] 
                matlab_fpa = params['fpa']        
                matlab_roll = params['roll']
                # Change the matlab roatations to the ritatoin of ENU that Gazebo uses:
                enu_yaw = (math.pi / 2.0) - matlab_heading
                enu_pitch = -1 * matlab_fpa
                enu_roll = matlab_roll
                # Make them strings:
                spawn_Y = str(enu_yaw)
                spawn_P = str(enu_pitch)
                spawn_R = str(enu_roll)


    # Publish the robot in the robot state:
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        namespace = robot_name,
        parameters = [{
            'frame_prefix': robot_name + '/',
            'use_sim_time': True,
            'robot_description': Command([
                'xacro ', robot_xacro_path,
                ' robot_name:=', robot_name,
                ' robot_scale:=', robot_scale,
                ' camera_num:=', camera_num,
                ' world_name:=', world_name,
                ' initial_alt:=', spawn_z,
                ' fog_lifetime:=', fog_lifetime,
                ' fog_rate:=', fog_rate,
                ' enable_fog:=', enable_fog,
            ])
        }],
    )

    # Joint state publisher to enable transform tree generation
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[{'use_sim_time': True}]
    )



    # Input the robot description in Gazebo Sim in a specific position:
    start_gz_spawner_cmd = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        name='create_entity',
        namespace=robot_name,
        output='screen',
        arguments=[
            '-name', robot_name,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y,
            '-topic', 'robot_description',
            '-allow_renaming', 'true'
        ]
    )

    return [robot_state_publisher_node,joint_state_publisher_node,start_gz_spawner_cmd]



def generate_launch_description():
    return LaunchDescription([
        # Make sure to declare the uav_id so the OpaqueFunction can find it
        DeclareLaunchArgument('uav_id', default_value='1'),
        DeclareLaunchArgument('type_uav', default_value='fixed_wing'),
        DeclareLaunchArgument('robot_name', default_value='airplane_1'),
        DeclareLaunchArgument('robot_scale', default_value='1.0'),
        DeclareLaunchArgument('camera_num', default_value='1'),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        OpaqueFunction(function=launch_setup)
    ])

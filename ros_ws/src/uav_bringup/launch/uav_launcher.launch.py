import os

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
            '-R', '0', '-P', '0', '-Y', '0',
            '-topic', 'robot_description',
            '-allow_renaming', 'true'
        ]
    )

    return [robot_state_publisher_node,joint_state_publisher_node,start_gz_spawner_cmd]



def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


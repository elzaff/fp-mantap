import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess

from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('fp-mantap')
    default_model_path = os.path.join(pkg_share, 'description/diff_drive_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')
    world_path = os.path.join(pkg_share, 'worlds/my_world.sdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bot', 
                   '-topic', 'robot_description', 
                   '-x', '7.3', 
                   '-y', '3.7',
                   '-z', '0.2',
                   '-Y', '3.14'
                  ],
        output='screen'
    )
    
    # Node untuk membaca input dari joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # Pastikan perangkat joystick sesuai
    )
    
    # Node untuk menggerakkan robot menggunakan joystick
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/joy_config.yaml')]  # Menambahkan path ke file konfigurasi YAML
    )

    pid_controller_node = Node(
    package='fp-mantap',
    executable='pid_controller.py',
    name='pid_controller',
    output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        joy_node,  # Menambahkan node joystick
        teleop_twist_joy_node,  # Menambahkan node teleop_twist_joy
        pid_controller_node  # Add the PID controller node
    ])
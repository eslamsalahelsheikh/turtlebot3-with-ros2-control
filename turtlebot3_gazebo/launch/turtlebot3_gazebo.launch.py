import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    pkg_box_car_description = get_package_share_directory(
        'turtlebot3_description')
    xacro_file = os.path.join(pkg_box_car_description, 'urdf/', 'turtlebot3.urdf.xacro')
    assert os.path.exists(
        xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix('turtlebot3_description')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'


    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    turtlebot3_gazebo = os.path.join(
        pkg_turtlebot3_gazebo,
        'worlds',
        'empty_world.world')

    # gazebo launch
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': turtlebot3_gazebo}.items(),
             )
    # controllers launch
    pkg_control_bringup = get_package_share_directory('turtlebot3_bringup')
    control_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control_bringup, 'launch', 'turtlebot3.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'empty_world.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'turtlebot3'],
                        output='screen'),
        gazebo,
        control_bringup,
    ])

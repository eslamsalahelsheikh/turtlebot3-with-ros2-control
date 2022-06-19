from launch import LaunchDescription
from launch.actions import RegisterEventHandler,ExecuteProcess, IncludeLaunchDescription,DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro, os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

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
        
    # Get URDF via xacro
    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('turtlebot3_description'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'turtlebot3.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_sim_time' : 'true'})
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_description"), "rviz", "model.rviz"]
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[params],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
        )

    robot_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'turtlebot3_base_controller'],
        output='screen'
    )    
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-topic', 'robot_description',
                                    '-entity', 'turtlebot3'],
                            output='screen')


    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'empty_world.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            'use_sim_time',
                default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument(
                'robot_description',
                default_value=xacro_file,
                description='robot_description file'),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[rviz_node],
                )
            ),
        gazebo,
            robot_state_pub_node,
            spawn_entity   
    ])

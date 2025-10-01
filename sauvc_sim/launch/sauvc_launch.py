import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_sauvc_sim = get_package_share_directory('sauvc_sim')
    urdf_file_path = os.path.join(pkg_sauvc_sim, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_sauvc_sim, 'rviz', 'default.rviz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bridge_config_file = os.path.join(pkg_sauvc_sim, 'config', 'bridge.yaml')

    # FINAL FIX: The xacro command must be a list of two separate strings.
    robot_description = ParameterValue(
        Command(['xacro',' ', urdf_file_path]), 
        value_type=str
    )

    # Set the Gazebo resource path
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH']
        new_gz_resource_path = os.path.join(pkg_sauvc_sim, 'models') + ':' + gz_resource_path
    else:
        new_gz_resource_path = os.path.join(pkg_sauvc_sim, 'models')

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    world_path = os.path.join(pkg_sauvc_sim, 'worlds', 'sauvc25.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'auv',
            '-x', '8.42',
            '-y', '-10.66',
            '-z', '-0.77',
            '-R', '-0.0008',
            '-P', '0.0',
            '-Y', '1.61'
        ],
        output='screen'
    )

    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        # Remap the auto-generated ROS topics to the ones we want
        # remappings=[
        #     ('/demo/camera', '/camera/image'),
        #     ('/demo/depth_camera', '/depth/image')
        # ],
        output='screen'
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_file}'
        ],
        output='screen'
    )

    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[LaunchConfiguration('image_topic')],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        robot_state_publisher_node,
        spawn_robot_node,
        image_bridge_node,
        bridge,
        start_rviz_node
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    # Set urdf package path, replace it with your package
    pkg_urdf_path = FindPackageShare('engineer_description')
    default_rviz_config_path = PathJoinSubstitution([pkg_urdf_path, 'rviz', 'engineer_description.rviz'])
    # Show joint state publisher GUI for joints
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')

    # RViz config file path
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file')
    
    # URDF model path within your package
    model_arg = DeclareLaunchArgument(
        'model', default_value='engineer_total.urdf.xacro',
        description='Name of the URDF description to load'
    )
    # publish static_transform
    # 把车转正
    # 没用了，新模型已经正了
    # static_transform_cmd = ExecuteProcess(
    #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0', '--y', '0', '--z', '0',
    #          '--roll', '1.5708', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'base_link'],
    #     output='screen'
    # )
    # Use built-in ROS2 URDF launch package with our own arguments
    urdf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'engineer_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', LaunchConfiguration('model')]),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    )
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gui_arg)
    launchDescriptionObject.add_action(rviz_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(urdf)
    # launchDescriptionObject.add_action(static_transform_cmd)
    return launchDescriptionObject
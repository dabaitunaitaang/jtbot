import os
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name = 'jtbot_navigation2'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name) 
    use_sim_time = LaunchConfiguration('use_sim_time')
    remappings = [('/odometry/filtered', '/odom')]
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='仿真用true')

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    ld =  launch.LaunchDescription()
                                            
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_localization_node)                                           
    return ld
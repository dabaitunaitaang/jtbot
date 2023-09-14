import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port_name = LaunchConfiguration('port_name')
    remappings = [('/odom', '/odom_diff')]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='仿真用true')

    declare_port_name_cmd = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyTHS1',
        description='声明端口号')

    foc_driver_node = launch_ros.actions.Node(
        package='jtbot_foc_driver',
        executable='jtbot_foc_driver',
        name='jtbot_foc_driver',
        output='screen',
        parameters=[{'port_name': port_name},
        {'use_sim_time': use_sim_time}],
        remappings=remappings,  
        
    )

    ld =  launch.LaunchDescription()
    ld.add_action(declare_port_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(foc_driver_node)                                           
    return ld
    
   

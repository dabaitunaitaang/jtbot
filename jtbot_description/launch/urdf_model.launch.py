import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# 机器人模型启动程序，与实体机器人一起启动。
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'jtbot_description'    #模型包名
    ld = LaunchDescription()

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_file = os.path.join(pkg_share, 'urdf/jtbot_base.urdf') #模型文件路径

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

     # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file]
    ) 
  
 
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_robot_state_publisher_cmd)


    return ld

#下面是xacro模型文件启动方法
# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# import xacro
# # 机器人模型启动程序，与实体机器人一起启动。
# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     package_name = 'jtbot_description'    #模型包名
#     ld = LaunchDescription()

#     pkg_share = FindPackageShare(package=package_name).find(package_name) 
#     urdf_xacro_file = os.path.join(pkg_share, 'urdf/jtbot_base.urdf.xacro') #xacro模型文件路径
#     #解析xacro模型文件
#     doc = xacro.parse(open(urdf_xacro_file))
#     xacro.process_doc(doc)
#     params = {'robot_description': doc.toxml()}

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation (Gazebo) clock if true')

#      # Start Robot State publisher
#     start_robot_state_publisher_cmd = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[params,{'use_sim_time': use_sim_time}],
#         # remappings=remappings,
#     ) 
#     # Start joint State publisher
#     start_joint_state_publisher_cmd = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         output='screen',
#         parameters=[
#             params,
#             {'use_sim_time': use_sim_time}],
        
#     ) 
 
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(start_robot_state_publisher_cmd)
#     ld.add_action(start_joint_state_publisher_cmd)

#     return ld




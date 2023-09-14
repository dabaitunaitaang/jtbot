

# 此launch文件是机器人仿真程序，包含 gazebo启动，机器人仿真生成，机器人模型状态发布
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    robot_name_in_model = 'jtbot'         #机器人模型名字
    package_name = 'jtbot_description'    #模型包名

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    gazebo_world_path = os.path.join(pkg_share, 'world/jt.world')        #世界仿真文件路径
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mrviz2.rviz')  #rviz配置文件路径
    urdf_xacro_file = os.path.join(pkg_share, 'urdf/jtbot_base.urdf.xacro') #xacro模型文件路径
    #解析xacro模型文件
    doc = xacro.parse(open(urdf_xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 开启ros Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo',
        '--verbose', 
        gazebo_world_path,
        #初始化gazebo-ros
         '-s', 'libgazebo_ros_init.so', 
        #gazebo产卵程序启动  
         '-s', 'libgazebo_ros_factory.so',    
         ],
        output='screen')
 

    # gazebo内生成机器人
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        )
	
       # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=remappings,
        parameters=[params,{'use_sim_time': use_sim_time}]
    )   

    ## Launch RViz
    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', default_rviz_config_path]
    #     )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_rviz_cmd)


    return ld

#======================gazebo launch文件编写示例=======================

"""gazebo.launch.py 使用命令行参数启动Gazebo服务器和客户端."""

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch.substitutions import ThisLaunchFileDir

# def generate_launch_description():

#     return LaunchDescription([
#         DeclareLaunchArgument('gui', default_value='true',
#                               description='Set to "false" to run headless.'),

#         DeclareLaunchArgument('server', default_value='true',
#                               description='Set to "false" not to run gzserver.'),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([ '/opt/ros/foxy/share/gazebo_ros/launch/gzserver.launch.py']),
#             condition=IfCondition(LaunchConfiguration('server'))
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(['/opt/ros/foxy/share/gazebo_ros/launch/gzclient.launch.py']),
#             condition=IfCondition(LaunchConfiguration('gui'))
#         ),
#     ])

""" Demo for spawn_entity.启动 Gazebo 生成一个模型 """
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir
# from launch_ros.actions import Node


# def generate_launch_description():
#     gazebo = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(['/opt/ros/foxy/share/gazebo_ros/launch/gazebo.launch.py']),
#              )

#     # GAZEBO_MODEL_PATH 必须正确设置Gazebo才能找到模型
#     spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
#                         arguments=['-entity', 'demo', '-database', 'my_robot'],
#                         output='screen')

#     return LaunchDescription([
#         gazebo,

#         spawn_entity,
#     ])

"""打开gazebo世界"""
# import os

# from launch import LaunchDescription

# from launch.actions import IncludeLaunchDescription

# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch.substitutions import ThisLaunchFileDir

# from launch_ros.actions import Node

# from launch.actions import ExecuteProcess



# def generate_launch_description():

#     world_file_name = 'contact.world'

#     world = os.path.join('/usr/share/gazebo-11', 'worlds', world_file_name)

#     gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world,
#     #  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'
#      ], output='screen')

#     return LaunchDescription([

#     gazebo

#     ])



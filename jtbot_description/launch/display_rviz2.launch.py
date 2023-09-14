"""在rviz中显示模型"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import xacro


# 机器人模型启动程序，与实体机器人一起启动。
def generate_launch_description():

    package_name = 'jtbot_description'    #模型包名
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_xacro_file = os.path.join(pkg_share, 'urdf/jtbot_base.urdf.xacro') #xacro模型文件路径
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mrviz2.rviz')  #rviz配置文件路径

    #解析xacro模型文件
    doc = xacro.parse(open(urdf_xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 创建Robot State publisher节点
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    ) 

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',      
    )

    # RViz2启动节点
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
        )

    ld = LaunchDescription()                       #创建LaunchDescription对象

    ld.add_action(start_robot_state_publisher_cmd) #启动Robot State publisher节点
    ld.add_action(joint_state_publisher_node)      #启动关节节点
    ld.add_action(start_rviz_cmd)                  #启动rviz2


    return ld
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    #机器人模型launch文件
    moxing_launch=os.path.join(get_package_share_directory('jtbot_description'),'launch','urdf_model.launch.py')
   
    #底盘速度控制节点launch文件
    jtbot_foc_driver_launch=os.path.join(get_package_share_directory('jtbot_foc_driver'),'launch','jtbot_foc_driver.launch.py')
    
    #雷达包launch文件
    lslidar_driver_launch=os.path.join(get_package_share_directory('lslidar_driver'),'launch','lsm10_uart_launch.py')
    
    #imu包launch文件
    imu_launch=os.path.join(get_package_share_directory('imu_get_cpp'),'launch','imu_cpp.launch.py')
      
    #robot_localization包ekf launch文件
    odom_ekf_launch=os.path.join(get_package_share_directory('jtbot_navigation2'),'launch','odom_ekf.launch.py')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        
        #启动机器人模型launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([moxing_launch]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),    
        ),

         #启动imu节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_launch]),
            ),

           #启动雷达launch 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lslidar_driver_launch]),
          launch_arguments={'use_sim_time': use_sim_time}.items(),    
            ),  

        #启动底盘速度控制节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([jtbot_foc_driver_launch]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),  
        ),

        #启动里程计 imu融合节点
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([odom_ekf_launch]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),  
        ),


            ])
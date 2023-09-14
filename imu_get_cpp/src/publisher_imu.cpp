#include <chrono>
#include <math.h>
#include "serial/serial.h" //导入串口库，ros2不带串口库，需要单独安装
#include <memory.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "transform.hpp"

serial::Serial ser;

using namespace std::chrono_literals;

class publisher_imu_node : public rclcpp::Node// 创建imu发布类继承自rclcpp::Node
{
private:
    std::string port;   //端口名
    int baudrate;       //波特率查看硬件说明
    transform_imu imu_fetch; //创建imu获取对象
    rclcpp::TimerBase::SharedPtr timer_;   //创建定时器
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu; // 创建发布者
public:
    publisher_imu_node()
        : Node("publisher_imu_node")  //构造函数
    {
        port = "/dev/ttyimu";  // 根据自己的主机修改串口号
        baudrate = 115200;      // 波特率，imu在波特率115200返回数据时间大概是1ms,9600下大概是10ms
        try
        {
            ser.setPort(port);           //设置端口
            ser.setBaudrate(baudrate);   //设置波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1); //设置超时
            ser.setTimeout(to);
            ser.open();  //打开串口
        }
        catch (serial::IOException &e)
        {
            RCLCPP_INFO(this->get_logger(), "打开imu端口失败,请检查端口连接！\n ");
            return;
        }

        // if (ser.isOpen())
        // {
        //     RCLCPP_INFO(this->get_logger(), "imu串口打开成功！");          
        //     //unsigned char cmd_buffer[5] ={0xA4,0x03,0x08,0x23,0xD2}; //JY-95T启用35个寄存器 需要发送的串口包命令
        //     //unsigned char cmd_buffer[5] ={0xA4,0x06,0x05,0x55,0x04}; //储存设置，掉电保存
        //     //ser.write(cmd_buffer,5);  //JY-95T可以根据命令选择工作方式
        //     //RCLCPP_INFO(this->get_logger(), "向imu发出配置命令: %x %x %x %x %x\n",cmd_buffer[0],cmd_buffer[1],cmd_buffer[2],cmd_buffer[3],cmd_buffer[4]);
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "imu串口打开失败，检查串口连接");
        //     return;
        // }

        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 20);//创建了话题 /imu_data
        
        // 创建定时器，100ms为周期，定时发布，这个时间设置超过200毫秒发布的数据卡顿
        timer_ = this->create_wall_timer(100ms, std::bind(&publisher_imu_node::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "imu初始化成功，/imu_data话题开始发布！\n");
    }

public:
    void timer_callback()   //定时器的回调函数，每20毫秒回调一次
    {
        int count = ser.available(); // 读取到缓存区数据的字节数
        
        if (count > 0)
        {
          
            int num;
            
            rclcpp::Time now = this->get_clock()->now(); // 获取时间戳

            std::vector<unsigned char> read_buf(count);//这里定义无符号，是因为read函数的形参是无符号
           
            num = ser.read(&read_buf[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数

            imu_fetch.FetchData(read_buf, num);

            sensor_msgs::msg::Imu imu_data;         
            
            //----------------imu data填充数据----------------
            imu_data.header.stamp = now;
            imu_data.header.frame_id = "imu_link";
            //imu_data.header.frame_id = "map";
            //加速度
            imu_data.linear_acceleration.x = imu_fetch.acc_x;
            imu_data.linear_acceleration.y = imu_fetch.acc_y;
            imu_data.linear_acceleration.z = imu_fetch.acc_z;
            //角速度
            imu_data.angular_velocity.x = imu_fetch.gyro_x ;
            imu_data.angular_velocity.y = imu_fetch.gyro_y ;
            imu_data.angular_velocity.z = imu_fetch.gyro_z ;
            //四元数
            imu_data.orientation.x = imu_fetch.quat_Q0;
            imu_data.orientation.y = imu_fetch.quat_Q1;
            imu_data.orientation.z = imu_fetch.quat_Q2;
            imu_data.orientation.w = imu_fetch.quat_Q3;
            
            pub_imu->publish(imu_data);  //向话题放数据
             /*
            Author: Mao Haiqing 
            Time: 2023.7.6
            description: 读取IMU数据
            */
          
        }
       
    }   
   
   
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<publisher_imu_node>();//创建对应节点的共享指针对象
    rclcpp::spin(node);   //运行节点，并检测退出信号
    printf("imu线程退出\n");
    rclcpp::shutdown();
    return 0;
}

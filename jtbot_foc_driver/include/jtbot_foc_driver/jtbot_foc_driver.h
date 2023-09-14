#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <nav_msgs/msg/odometry.hpp>    //里程计接口
#include <geometry_msgs/msg/twist.hpp>  

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <std_msgs/msg/float32.hpp>

#include <string>
#include <vector>
#include <math.h>
#include <serial/serial.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
//调整到间隔。
//如果amt小于low，则返回low；如果amt大于high，则返回high；否则，返回amt。它通常用于将值约束在某个范围内。
#define BATTERY_RATE 1   //电池电量

#define START_FRAME 0xABCD  //串口数据帧头

#define MAX_RPM 366  //最大转速
#define FOC_MAX_PWM 1000  //foc 最大
#define WHEEL_DIAMETER 0.155 //车轮直径

#define LR_WHEELS_DISTANCE 0.517  //左右轮距
#define FR_WHEELS_DISTANCE 0.0    //前后轮距
#define PI 3.1415926

typedef struct
{
  uint16_t start;     //0XABCD
  int16_t rpmR;       //右轮转速
  int16_t rpmL;       //左轮转速
  int16_t batVoltage; //主板电压
  int16_t boardTemp;  //主板温度
  int16_t curL_DC;    //左电机电流
  int16_t curR_DC;    //右电机电流
  uint16_t checksum;  //校验和
} JtbotFeedback;    //速控板反馈

typedef struct
{
  uint16_t start;
  int16_t mSpeedR;
  int16_t mSpeedL;
  uint16_t checksum;
} JtbotCommand;   //左右电机速度控制

typedef struct
{
  int motor1;
  int motor2;
} MotorRPM;  //“r”转；“p”是“每”的意思；“m”是“分钟”左右电机 转速/分钟

typedef struct
{
  float linear_x;
  float linear_y;
  float angular_z;
} Velocities;//速度结构体

class JtbotFocDriver : public rclcpp::Node
{
public:
  JtbotFocDriver();  //构造函数
  ~JtbotFocDriver(); //析构函数
  

private:
  rclcpp::TimerBase::SharedPtr topic_timer_;  //声明定时器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_; //声明速度接收

  
  bool ReadFormSerial();  //读串口

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);  //收到速度控制 回调函数
  void RosNodeTopicCallback(); //定时器定时发布 回调函数
  void SetVelocity(double x, double y, double angular_z);  //设置速度

  void calculateRPM(float linear_x, float linear_y, float angular_z);  //计算转速
  void getVelocities(int rpm1, int rpm2); //获取速度
  int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max); //pwm和速度映射
  //void PublisherMotorVel();  
  
  void publisherBattery();  //发布显示电量
  std_msgs::msg::Float32 raw_battery_msg;//声明电量消息
  int max_rpm_;  //最大转速
  float wheel_diameter_;  //车轮直径
  float wheels_x_distance_;  //车轮左右距离
  float wheels_y_distance_;  //车轮前后距离
  float wheel_circumference_;  //车轮周长

  serial::Serial Robot_Serial;  //创建串口对象

  JtbotFeedback Feedback;  //速控版反馈
  JtbotCommand Command;    //控制命令
  Velocities vel;//速度结构体对象vel
  MotorRPM req_rpm;  //电机转速

  rclcpp::Time now_;

  std::string port_name_; //端口名
  std::string rxdata;

  int baud_rate_;  //波特率


rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;  //声明odom发布者
std::shared_ptr<tf2_ros::TransformBroadcaster> odom_transform_broadcaster_; //声明tf2发布者
 rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;  //声明电量发布者
void PublisherOdom(); //声明发布odom函数

nav_msgs::msg::Odometry odom_;  //声明odom消息接口
geometry_msgs::msg::TransformStamped odom_transform_; //声明tf2消息接口
rclcpp::Time last_odom_time_;
float odom_theta_; //小车角度
float x_pos_;   //小车x坐标
float y_pos_;   //小车y坐标
};

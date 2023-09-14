#include "jtbot_foc_driver/jtbot_foc_driver.h"
using namespace std::chrono_literals;

JtbotFocDriver::JtbotFocDriver() : Node("jtbot_foc_driver"),
                                   odom_theta_(0), x_pos_(0), y_pos_(0) // 构造函数，初始化成员变量
{
    wheel_diameter_ = WHEEL_DIAMETER;            // 轮子直径
    wheel_circumference_ = PI * wheel_diameter_; // 轮子周长
    max_rpm_ = MAX_RPM;                          // 轮子最大转速
    wheels_x_distance_ = FR_WHEELS_DISTANCE;     // 2轮差速 前后轮子距离 0
    wheels_y_distance_ = LR_WHEELS_DISTANCE;     // 左右轮子距离
    now_ = now();

    declare_parameter("port_name", std::string("/dev/ttyTHS1")); // 根据实际端口修改
    get_parameter("port_name", port_name_);
    declare_parameter("baud_rate", 115200);
    get_parameter("baud_rate", baud_rate_);
    // 发布电池电量
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery", 10);
    // 定时10毫秒发布电池和速度 odom 和 tf2
    topic_timer_ = this->create_wall_timer(10ms, std::bind(&JtbotFocDriver::RosNodeTopicCallback, this));
    // 接收速度话题 回调函数发布控制命令
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&JtbotFocDriver::cmd_vel_callback, this, std::placeholders::_1));

    // odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_diff", 20);  //创建odom发布者
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);       // 创建odom发布者
    odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); // 创建tf2发布者
    last_odom_time_ = now();

    odom_.header.frame_id = "odom";
    // odom_.child_frame_id = "base_footprint";
    odom_.child_frame_id = "base_link";

    odom_transform_.header.frame_id = "odom";
    // odom_transform_.child_frame_id = "base_footprint";
    odom_transform_.child_frame_id = "base_link";

    /**打开串口设备**/
    try
    {
        Robot_Serial.setPort(port_name_);
        Robot_Serial.setBaudrate(baud_rate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(2000);
        Robot_Serial.setTimeout(to);
        Robot_Serial.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_INFO(this->get_logger(), "foc速控板打开失败，请检查串口是否连接");
    }

    // if (Robot_Serial.isOpen())
    // {
    //     RCLCPP_INFO(this->get_logger(), "速控板串口打开成功");
    // }
    // else
    // {
    // }
}

JtbotFocDriver::~JtbotFocDriver()
{
    Robot_Serial.close();
}

void JtbotFocDriver::RosNodeTopicCallback() // ros节点话题回调
{
    if (true == ReadFormSerial())
    {
        publisherBattery(); // 发布电压 低于33伏发布充电警告

        PublisherOdom(); // 发布odom
    }
}

bool JtbotFocDriver::ReadFormSerial() // 读串口
{
    if (Robot_Serial.available())
    {
        rxdata = Robot_Serial.read(Robot_Serial.available());

        if (rxdata.size() == sizeof(JtbotFeedback))
        {
            memcpy(&Feedback, rxdata.c_str(), sizeof(JtbotFeedback));
            // printf("Feedbac左轮转速 %hd 右轮转速 %hd 电压 %lf 主板温度 %lf\n",Feedback.rpmL,Feedback.rpmR,double(Feedback.batVoltage)/100,double(Feedback.boardTemp)/10);
            uint16_t checksum = (uint16_t)(Feedback.start ^ Feedback.rpmR ^ Feedback.rpmL ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.curL_DC ^ Feedback.curR_DC);
            if ((Feedback.start == START_FRAME) && (checksum == Feedback.checksum))
                return true;
            else
                return false;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "速控板读取数据失败11111");
        return false;
    }
}

/*cmd_vel Subscriber的回调函数*/
void JtbotFocDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    SetVelocity(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
}

/*底盘速度发送函数*/
void JtbotFocDriver::SetVelocity(double x, double y, double angular_z)
{
    calculateRPM(x, y, angular_z);
    // 把速度映射为foc波形
    int16_t mSpeedL = map(req_rpm.motor1, -max_rpm_, max_rpm_, -FOC_MAX_PWM, FOC_MAX_PWM);
    int16_t mSpeedR = map(req_rpm.motor2, -max_rpm_, max_rpm_, -FOC_MAX_PWM, FOC_MAX_PWM);
    // 根据收到的cmd_vel计算左右轮的速度，装填数据，写人串口
    Command.start = (uint16_t)START_FRAME;
    Command.mSpeedR = (uint16_t)mSpeedR;
    Command.mSpeedL = (uint16_t)mSpeedL;
    Command.checksum = (uint16_t)(Command.start ^ Command.mSpeedR ^ Command.mSpeedL);

    if (Robot_Serial.write((uint8_t *)&Command, sizeof(Command)) <= 0)
    {
        RCLCPP_INFO(this->get_logger(), "速控板写入速度控制数据失败，请检查");
    }
}

void JtbotFocDriver::publisherBattery() // 发布显示电压
{

    if ((now() - now_).seconds() > 1 / BATTERY_RATE)
    {
        raw_battery_msg.data = (float)Feedback.batVoltage / 100;
        if (raw_battery_msg.data < 35)
        {
            RCLCPP_WARN(this->get_logger(), "当前电压: %f ,电压偏低，请充电!!", raw_battery_msg.data);
            battery_pub_->publish(raw_battery_msg);
            now_ = now();
        }
    }
}

void JtbotFocDriver::PublisherOdom() // 发布odom
{
    getVelocities(Feedback.rpmL, Feedback.rpmR); // 获取速度，数据填充到vel结构体
    rclcpp::Time current_time = now();
    odom_.header.stamp = current_time;
    // x线速度
    auto linear_velocity_x_ = vel.linear_x;
    // y线速度
    auto linear_velocity_y_ = vel.linear_y;
    // 角速度
    auto angular_velocity_z_ = vel.angular_z;

    rclcpp::Duration vel_dt_ = current_time - last_odom_time_;

    last_odom_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_.nanoseconds() / 1e9;               // radians
    double delta_x = (linear_velocity_x_ * cos(odom_theta_)) * vel_dt_.nanoseconds() / 1e9; // m
    double delta_y = (linear_velocity_x_ * sin(odom_theta_)) * vel_dt_.nanoseconds() / 1e9; // m

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    odom_theta_ += delta_heading;
    tf2::Quaternion odom_q;           // 声明四元数
    odom_q.setRPY(0, 0, odom_theta_); // 欧拉角换算四元数

    // 相对于父坐标系位置
    odom_.pose.pose.position.x = x_pos_;
    odom_.pose.pose.position.y = y_pos_;
    odom_.pose.pose.position.z = 0.0;
    // 相对于父坐标系姿态四元数
    odom_.pose.pose.orientation.x = odom_q.x();
    odom_.pose.pose.orientation.y = odom_q.y();
    odom_.pose.pose.orientation.z = odom_q.z();
    odom_.pose.pose.orientation.w = odom_q.w();
    // 姿态协方差
    //  odom_.pose.covariance[0] = 0.001;
    //  odom_.pose.covariance[7] = 0.001;
    //  odom_.pose.covariance[35] = 0.001;
    // 相对于子坐标系线速度和角速度
    odom_.twist.twist.linear.x = linear_velocity_x_;
    odom_.twist.twist.linear.y = linear_velocity_y_;
    odom_.twist.twist.angular.z = angular_velocity_z_;

    // 协方差分为静止和运动：
    // pose：

    // if(linear_velocity_x_==0){
    //     //pose静止：
    //     odom_.pose.covariance[0] = 1e-9;
    //     odom_.pose.covariance[7] = 1e-3;
    //     odom_.pose.covariance[8] = 1e-9;
    //     odom_.pose.covariance[14] = 1e6;
    //     odom_.pose.covariance[21] = 1e6;
    //     odom_.pose.covariance[28] = 1e6;
    //     odom_.pose.covariance[35] = 1e-9;
    //     //twist静止：
    //     odom_.twist.covariance[0] = 1e-9;
    //     odom_.twist.covariance[7] = 1e-3;
    //     odom_.twist.covariance[8] = 1e-9;
    //     odom_.twist.covariance[14] = 1e6;
    //     odom_.twist.covariance[21] = 1e6;
    //     odom_.twist.covariance[28] = 1e6;
    //     odom_.twist.covariance[35] = 1e-9;
    // }
    // else{
    //      //pose运动：
    //     odom_.pose.covariance[0] = 1e-3;
    //     odom_.pose.covariance[7] = 1e-3;
    //     odom_.pose.covariance[8] = 0.0;
    //     odom_.pose.covariance[14] = 1e6;
    //     odom_.pose.covariance[21] = 1e6;
    //     odom_.pose.covariance[28] = 1e6;
    //     odom_.pose.covariance[35] = 1e3;
    //     //twist运动：
    //     odom_.twist.covariance[0] = 1e-3;
    //     odom_.twist.covariance[7] = 1e-3;
    //     odom_.twist.covariance[8] = 0.0;
    //     odom_.twist.covariance[14] = 1e6;
    //     odom_.twist.covariance[21] = 1e6;
    //     odom_.twist.covariance[28] = 1e6;
    //     odom_.twist.covariance[35] = 1e3;
    // }

    odom_.header.stamp = now();
    odom_publisher_->publish(odom_);

    // printf("odom_.pose x y z %f %f %f \n",x_pos_,y_pos_,odom_theta_);

    // printf("odom_.twist x y z %f %f %f \n",linear_velocity_x_,linear_velocity_y_,angular_velocity_z_);

    odom_transform_.header.stamp = last_odom_time_;
    odom_transform_.transform.translation.x = x_pos_;
    odom_transform_.transform.translation.y = y_pos_;
    odom_transform_.transform.rotation.x = odom_q.x();
    odom_transform_.transform.rotation.y = odom_q.y();
    odom_transform_.transform.rotation.z = odom_q.z();
    odom_transform_.transform.rotation.w = odom_q.w();
    // odom_transform_broadcaster_->sendTransform(odom_transform_);  //开odom imu融合需要关闭/tf话题
    // printf("now last_odom_time_%ld %ld %ld\n",now(),current_time,last_odom_time_);
}

// 计算转速
void JtbotFocDriver::calculateRPM(float linear_x, float linear_y, float angular_z)
{
    float linear_vel_x_mins;
    float linear_vel_y_mins;
    float angular_vel_z_mins;
    float tangential_vel;
    float x_rpm;
    float y_rpm;
    float tan_rpm;

    // 转换米/秒 至 米/分
    linear_vel_x_mins = linear_x * 60;
    linear_vel_y_mins = linear_y * 60;

    // 转换 rad/秒 to rad/分
    angular_vel_z_mins = angular_z * 60;

    tangential_vel = angular_vel_z_mins * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2));

    x_rpm = linear_vel_x_mins / wheel_circumference_;
    y_rpm = linear_vel_y_mins / wheel_circumference_;
    tan_rpm = tangential_vel / wheel_circumference_;

    req_rpm.motor1 = int16_t(x_rpm - y_rpm - tan_rpm);
    req_rpm.motor1 = constrain(req_rpm.motor1, -max_rpm_, max_rpm_);
    // front-right motor
    req_rpm.motor2 = -int16_t(x_rpm + y_rpm + tan_rpm);
    req_rpm.motor2 = constrain(req_rpm.motor2, -max_rpm_, max_rpm_);
}

void JtbotFocDriver::getVelocities(int rpm1, int rpm2) // 获取速度，数据填充到vel结构体
{
    float average_rps_x;
    // float average_rps_y;
    float average_rps_a;

    // 将平均每分钟转数转换为每秒转数,小车向前走左轮为正，右轮为负
    average_rps_x = ((float)(rpm1 - rpm2) / 2) / 60;     // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    // 将y轴上的平均每分钟转数转换为每秒转数
    // average_rps_y = ((float)(-rpm1 + rpm2) / 2) / 60; // RPM

    vel.linear_y = 0;

    // 将平均每分钟转数转换为每秒转数
    average_rps_a = ((float)(rpm1 + rpm2) / 2) / 60;
    vel.angular_z = -(average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s
}

// 速度映射到foc波形
int16_t JtbotFocDriver::map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto driver = std::make_shared<JtbotFocDriver>();
    rclcpp::spin(driver);
    rclcpp::shutdown();
    return 0;
}

#include <string>
#include <ctype.h>
#include <float.h>
#include <math.h>
class transform_imu
{

public:
    
    double acc_x=0;//加速度
    double acc_y=0;
    double acc_z=0;
   
    double gyro_x=0; //陀螺仪角速度
    double gyro_y=0;
    double gyro_z=0;
   
    double angle_r=0; //欧拉角
    double angle_p=0;
    double angle_y=0;
    
    double temp=0;  //imu温度计 
   
    double mag_x=0;  //磁力计                                                               
    double mag_y=0;
    double mag_z=0;
   
    double quat_Q0=0; //四元数
    double quat_Q1=0;
    double quat_Q2=0;
    double quat_Q3=0;
    
    
public:
    transform_imu(){};  //默认构造函数
    ~transform_imu(){}; //析构函数

    // void FetchData(auto &data, int usLength) //获取数据 
    void FetchData(std::vector<unsigned char> &data, int usLength)
{   
    int index = 0;
    unsigned char sum=0;
    //for(int i = 0;i < usLength;i++){printf("%x ",data[i]);} //打印数据，查看收到的数据是否正确

    while (usLength >= 40)//一个完整数据帧40字节[0xA4,0x03,0x08,0x23,数据35字节，校验位] 
    {
            
        // 收到的数据开头4位：0xA4 帧头,0x03 读取,0x08 从8号寄存器开始,0x23 35个寄存器
        if (data[index] != 0xA4)//0xA4是协议头
        {
            index++;//指针(/索引)后移，继续找协议帧头
            usLength--;  
            continue;
        }
        //printf("找到数据包帧头，index是%d，uslength=%d\n",index,usLength);//每一次读取的数据长度
        
        if(data[index + 1] != 0x03)   //判断第二个数据是否正确
        {
           
           // printf("第2数据包内容不对，进入下个循环");
            usLength = usLength - 40;
            index += 40;
            continue;
        }   
        // printf("读imu数据\n");

         if(data[index + 2] != 0x08)  //判断第3个数据是否正确
        {
            // printf("第三数据包内容不对，进入下个循环");
            usLength = usLength - 40;
            index += 40;
            continue;
        }
         // printf("第08寄存器是数据\n");

         if(data[index + 3] != 0x23) //判断第4个数据是否正确
        {
            //printf("第四数据包内容不对，进入下个循环\n");
            usLength = usLength - 40;
            index += 40;
            continue;
        }
          //printf("35个数据包\n");  

            //0-39字节相加的值取低8位与第40位校验，参考GY-95T 九轴模块使用说明
          for(int i=index;i<index+40;i++) //前39为求和
          {
            sum=0+data[i]; 
            
          }
            
         if(sum != data[index+39]) //判断前39个数据的和是否与第40位相等
        {    
            usLength = usLength - 40;
            index += 40;
            printf("imu数据校验有误，请检查imu设置\n");
            continue;
        }
         // printf("校验和通过，数据没有问题\n"); 


        /*
        根据模块使用说明可知，一个完整数据帧40字节[0xA4,0x03,0x08,0x23,数据35字节，校验位]
        1.开头4位：0xA4 帧头,0x03 读取,0x08 数据位从8号寄存器开始,0x23 35个寄存器
        2.加速度位：4-9   
        3.陀螺仪位：10-15
        4.欧拉角：16-21
        5.磁场校准：22
        6.器件温度：23-24
        5.磁场：25-30
        6.四元数：31-38
        数据解析一定注意2个字节拼在一起用short类型，然后在转换成double类型，总体加小括号在作除法，
        否则short类型作除法会被截断降低精度，表现在图形显示界面会卡，数据不流畅。拼好的数据单位参考
        使用手册。
        */

        // 加速度
            acc_x = ((double)((short)(data[index + 5]<<8 | data[index + 4])))/2048 * 9.8;  //单位m/s^2
            acc_y = ((double)((short)(data[index + 7]<<8 | data[index + 6])))/2048 * 9.8;
            acc_z = ((double)((short)(data[index + 9]<<8 | data[index + 8])))/2048 * 9.8;

        // 陀螺仪角速度
            gyro_x = ((double)((short)(data[index + 11]<<8 | data[index + 10])))/16.4*3.1415926 / 180.0; //单位rad/sec
            gyro_y = ((double)((short)(data[index + 13]<<8 | data[index + 12])))/16.4*3.1415926 / 180.0;  
            gyro_z = ((double)((short)(data[index + 15]<<8 | data[index + 14])))/16.4*3.1415926 / 180.0;  
        
        //欧拉角
            // angle_r = ((double)((short)(data[index + 17]<<8 | data[index + 16])))/100 ; 
            // angle_p = ((double)((short)(data[index + 19]<<8 | data[index + 18])))/100 ; 
            // angle_y = ((double)((short)(data[index + 21]<<8 | data[index + 20])))/100 ; 
        
        //imu温度
            temp = ((double)((short)(data[index + 24]<<8 | data[index + 23])))/100 ; //单位摄氏度
        //磁力计
            // mag_x = ((double)((short)(data[index + 26]<<8 | data[index + 25]))); 
            // mag_y = ((double)((short)(data[index + 28]<<8 | data[index + 27]))); 
            // mag_z = ((double)((short)(data[index + 30]<<8 | data[index + 29])));
            

        //四元数  
            quat_Q0 = ((double)((short)(data[index + 32]<<8 | data[index + 31])))/10000 ; //无单位
            quat_Q1 = ((double)((short)(data[index + 34]<<8 | data[index + 33])))/10000 ; 
            quat_Q2 = ((double)((short)(data[index + 36]<<8 | data[index + 35])))/10000 ; 
            quat_Q3 = ((double)((short)(data[index + 38]<<8 | data[index + 37])))/10000 ; 

            //JY-95T imu传感器9轴，数据解析完成，具体用什么数据可以根据实际需要选用
          
            //调试结束后可以屏蔽以下printf
            //printf("加速度x y z: %lf %lf %lf\n",acc_x,acc_y,acc_z);
            //printf("角速度x y z: %lf %lf %lf\n",gyro_x,gyro_y,gyro_z);
            //printf("欧拉角r p y: %lf %lf %lf %lf\n",angle_r,angle_p,angle_y);
            //printf("imu温度计 %lf\n",temp);
            //printf("磁力计x y z: %lf %lf %lf\n",mag_x, mag_y,mag_z);
            //printf("四元数Q0 Q1 Q2 Q3: %lf %lf %lf %lf\n",quat_Q0,quat_Q1,quat_Q2,quat_Q3);
            /*
            Author: Mao Haiqing 
            Time: 2023.7.6
            description: 读取IMU数据
            */
            usLength = usLength - 40;
            index += 40;
    
     }
}

};

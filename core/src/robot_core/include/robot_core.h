#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "robot_msg/msg/control.hpp"
#include "robot_msg/msg/report.hpp"
#include "robot_msg/msg/position.hpp"

using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 30         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    9         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f

// extern sensor_msgs::msg::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
                                          0, 1e-3,    0,   0,   0,    0,
                                          0,    0,  1e6,   0,   0,    0,
                                          0,    0,    0, 1e6,   0,    0,
                                          0,    0,    0,   0, 1e6,    0,
                                          0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
                                          0, 1e-3, 1e-9,   0,   0,    0,
                                          0,    0,  1e6,   0,   0,    0,
                                          0,    0,    0, 1e6,   0,    0,
                                          0,    0,    0,   0, 1e6,    0,
                                          0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
                                          0, 1e-3,    0,   0,   0,    0,
                                          0,    0,  1e6,   0,   0,    0,
                                          0,    0,    0, 1e6,   0,    0,
                                          0,    0,    0,   0, 1e6,    0,
                                          0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
                                          0, 1e-3, 1e-9,   0,   0,    0,
                                          0,    0,  1e6,   0,   0,    0,
                                          0,    0,    0, 1e6,   0,    0,
                                          0,    0,    0,   0, 1e6,    0,
                                          0,    0,    0,   0,   0, 1e-9} ;


//IMU data structure
//IMU数据结构体
//typedef struct __MPU6050_DATA_
//{
//    short accele_x_data;
//    short accele_y_data;
//    short accele_z_data;
//    short gyros_x_data;
//    short gyros_y_data;
//    short gyros_z_data;
//
//}MPU6050_DATA;

typedef struct __POSITION_DATA_
{
    float x;
    float y;
    float z;
}POSITION_DATA;

//Data structure for throttle and servo
typedef struct __REPORT_DATA_
{
    float voltage;
    float throttle_a;
    float throttle_b;
    int servo;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_x;
    float angular_y;
    float angular_z;
}REPORT_DATA;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE];
    float Throttle_control;
    float Servo_control;
    unsigned char Frame_Tail;
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;
    float Y_speed;
    float Z_speed;
    float Power_Voltage;
    float A_throttle_measure;
    float B_throttle_measure;
    float Servo_measure;
    unsigned char Frame_Tail;
}RECEIVE_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class RobotCore : public rclcpp::Node

{
public:
    RobotCore();
    ~RobotCore(); //Destructor //析构函数
    void control();   //Loop control code //循环控制代码
    serial::Serial stm32Serial; //Declare a serial object //声明串口对象

private:

    float samplingTime;
    rclcpp::Subscription<robot_msg::msg::Control>::SharedPtr controlSubscriber;
    rclcpp::Publisher<robot_msg::msg::Report>::SharedPtr reportPublisher;
    rclcpp::Publisher<robot_msg::msg::Position>::SharedPtr positionPublisher;
    void controlCallback(const robot_msg::msg::Control::SharedPtr cmdControl);
    void publishReport();
    void publishPosition();
//    auto createQuaternionMsgFromYaw(double yaw);

    //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
    //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
    bool getSensorData();
    unsigned char checkSum(unsigned char countNumber,unsigned char mode); //BBC check function //BBC校验函数
    short imuTrans(uint8_t dataHigh,uint8_t dataLow);  //IMU data conversion read //IMU数据转化读取
    float odomTrans(uint8_t dataHigh,uint8_t dataLow); //Odometer data is converted to read //里程计数据转化读取

    string usartPortName; //Define the related variables //定义相关变量
    int serialBaudRate;      //Serial communication baud rate //串口通信波特率
    RECEIVE_DATA receiveData; //The serial port receives the data structure //串口接收数据结构体
    SEND_DATA sendData;       //The serial port sends the data structure //串口发送数据结构体
    POSITION_DATA position;

    REPORT_DATA reportData;
    float powerVoltage;       //Power supply voltage //电源电压
};


#endif


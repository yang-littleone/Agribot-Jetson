#ifndef TURN_ON_32CHASSIS_HPP_
#define TURN_ON_32CHASSIS_HPP_

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <iostream>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;

// Macro definition
// 宏定义
#define SEND_DATA_CHECK 1    // Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK 0    // Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER 0X7B    // Frame head //帧头
#define FRAME_TAIL 0X7D      // Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24 // The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE 11    // The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 3.1415926f        // PI //圆周率

// 与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
// 陀螺仪原始数据转换位弧度(rad)单位，gyro*(500/32768) (°/s) -> gyro*(500/32768)*PI/180 (rad/s) ->  gyro/3754.94
#define GYROSCOPE_RATIO 3754.94f

// 与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，accl*(2*9.8/32768) -> accl/1671.84
#define ACCEl_RATIO 1671.84f

// Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
// 协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                         0, 1e-3, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                          0, 1e-3, 1e-9, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                          0, 1e-3, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e3};

const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                           0, 1e-3, 1e-9, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e-9};

typedef struct
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;

} IMUData_t;

// 下位机向ROS发送数据的结构体

typedef struct
{
    uint8_t buffer[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;
    float Y_speed;
    float Z_speed;
    IMUData_t IMUData;
    float Power_Voltage;
    unsigned char Frame_Tail;
} ReceiveData_t;

typedef struct
{
    uint8_t buffer[SEND_DATA_SIZE];
    float X_speed;
    float Y_speed;
    float Z_speed;
    unsigned char Frame_Tail;
} SendData_t;

typedef struct
{
    float X;
    float Y;
    float Z;
} VelPosData_t;

class TurnOn32Chassis : public rclcpp::Node
{
public:
    TurnOn32Chassis(/* args */);
    ~TurnOn32Chassis();
    void run();

    serial::Serial STM32_Serial;

private:
    uint8_t check_sum(uint8_t *data, uint8_t len);

    bool receive_row_serialdata();
    short uchartoshort(uint8_t data_high, uint8_t data_low);
    void Publish_ImuSensor();
    void Publish_Odom();
    void Publish_Voltage();

    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    string port_name_ = "/dev/agribot_serial";

    int baud_rate_ = 115200;

    // Initialize the topic publisher //初始化话题发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub; // Initialize the topic subscriber //初始化话题订阅者

    ReceiveData_t receive_data_;
    SendData_t send_data_;
    sensor_msgs::msg::Imu Imu_Data_Pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_, current_time_;
    float sample_time_;
    VelPosData_t robot_vel_;
    VelPosData_t robot_pos_;
    float odom_x_scale_ = 1.0f, odom_y_scale = 1.0f, odom_z_scale_positive = 1.0f, odom_z_scale_negative = 1.0f; // 里程计修正参数
    string robot_frame_id, gyro_frame_id, odom_frame_id;                                                         // Define the related variables //定义相关变量
};

#endif // TURN_ON_32CHASSIS_HPP_
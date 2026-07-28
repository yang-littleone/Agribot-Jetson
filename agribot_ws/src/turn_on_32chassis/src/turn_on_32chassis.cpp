#include "turn_on_32chassis/turn_on_32chassis.hpp"
#include "turn_on_32chassis/Quaternion_Solution.h"

// 定义全局IMU消息变量
sensor_msgs::msg::Imu imu_msg_;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TurnOn32Chassis TurnOn32Chassis;
    TurnOn32Chassis.run();
    return 0;
}

TurnOn32Chassis::TurnOn32Chassis() : rclcpp::Node("TurnOn32Chassis")
{
    sample_time_ = 0;

    // 初始化结构体
    memset(&receive_data_, 0, sizeof(receive_data_));
    memset(&send_data_, 0, sizeof(send_data_));
    imu_msg_ = sensor_msgs::msg::Imu();
    memset(&robot_pos_, 0, sizeof(robot_pos_));
    memset(&robot_vel_, 0, sizeof(robot_vel_));

    // 注意：不要对 STM32_Serial 使用 memset，因为它是一个 C++ 对象

    // 声明参数
    this->declare_parameter<std::string>("port_name", "/dev/agribot_serial");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
    this->declare_parameter<std::string>("gyro_frame_id", "imu_link");
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);

    this->get_parameter<std::string>("port_name", port_name_);
    this->get_parameter<int>("baud_rate", baud_rate_);
    this->get_parameter("odom_frame_id", odom_frame_id);   // The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
    this->get_parameter("robot_frame_id", robot_frame_id); // The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
    this->get_parameter("gyro_frame_id", gyro_frame_id);   // IMU topics correspond to TF coordinates //IMU话题对应TF坐标
    this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
    if (cmd_vel_timeout_ < 0.0)
    {
        cmd_vel_timeout_ = 0.0;
    }
    last_cmd_vel_time_ = std::chrono::steady_clock::now();

    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 2); // Create the raw wheel odometry publisher //创建原始轮式里程计发布者
    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2); // Create an IMU topic publisher //创建IMU话题发布者

    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&TurnOn32Chassis::Cmd_Vel_Callback, this, std::placeholders::_1));

    try
    {
        STM32_Serial.setPort(port_name_);
        STM32_Serial.setBaudrate(baud_rate_);
        STM32_Serial.setParity(serial::parity_none);
        STM32_Serial.setStopbits(serial::stopbits_one);

        serial::Timeout _timeout = serial::Timeout::simpleTimeout(1000);
        STM32_Serial.setTimeout(_timeout);
        STM32_Serial.open();
        STM32_Serial.flushInput();
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", e.what());
    }

    if (STM32_Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "Port opened successfully");

        // 发送初始化命令
        // uint8_t tx_data[11] = {0x7B, 0x00, 0x00, 0x00, 0x75, 0x00, 0x00, 0x00, 0x00, 0x1F, 0X7D};
        // tx_data[9] = check_sum(tx_data, 9);
        // STM32_Serial.write(tx_data, sizeof(tx_data));

        // // 创建一个定时器来定期读取串口数据
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(500), // 每500毫秒读取一次
        //     std::bind(&TurnOn32Chassis::receive_row_serialdata, this));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    }

    if (cmd_vel_timeout_ > 0.0)
    {
        RCLCPP_INFO(this->get_logger(),
                    "cmd_vel watchdog enabled: %.2f s without a command sends zero velocity",
                    cmd_vel_timeout_);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "cmd_vel watchdog disabled");
    }
}

/**
 * @brief Convert two bytes to short
 * @param data_high High byte
 * @param data_low Low byte
 * @return short
 */
short TurnOn32Chassis::uchartoshort(uint8_t data_high, uint8_t data_low)
{

    short Temp_data = 0;
    Temp_data = (data_high << 8) | data_low;
    return Temp_data;
}

/**
 * @brief Receive data from serial port and parse it
 * @return bool
 */
bool TurnOn32Chassis::receive_row_serialdata()
{
    static uint8_t ReceiveDataTemp = 0, count = 0, status = 0;

    // 检查是否有数据可读
    if (STM32_Serial.available() > 0)
    {
        STM32_Serial.read(&ReceiveDataTemp, 1);
        // RCLCPP_INFO(this->get_logger(), "Received byte: 0x%02X count: %d status: %d", ReceiveDataTemp, count, status);

        if (count == 0 && ReceiveDataTemp == FRAME_HEADER && status == 0)
        {
            receive_data_.buffer[count] = ReceiveDataTemp;
            count++;
            status = 1;
        }
        else if (status == 1)
        {
            receive_data_.buffer[count] = ReceiveDataTemp;
            count++;
            if (count == 24)
            {
                status = 0;
                count = 0;
                if (receive_data_.buffer[23] == FRAME_TAIL && check_sum(receive_data_.buffer, 22) == receive_data_.buffer[22])
                {
                    // 解析解收到的数据
                    receive_data_.Frame_Header = receive_data_.buffer[0];
                    receive_data_.Flag_Stop = receive_data_.buffer[1];
                    receive_data_.X_speed = (float)uchartoshort(receive_data_.buffer[2], receive_data_.buffer[3]) / 1000.0f;
                    receive_data_.Y_speed = (float)uchartoshort(receive_data_.buffer[4], receive_data_.buffer[5]) / 1000.0f;
                    receive_data_.Z_speed = (float)uchartoshort(receive_data_.buffer[6], receive_data_.buffer[7]) / 1000.0f;
                    receive_data_.IMUData.accele_x_data = uchartoshort(receive_data_.buffer[8], receive_data_.buffer[9]);
                    receive_data_.IMUData.accele_y_data = uchartoshort(receive_data_.buffer[10], receive_data_.buffer[11]);
                    receive_data_.IMUData.accele_z_data = uchartoshort(receive_data_.buffer[12], receive_data_.buffer[13]);
                    receive_data_.IMUData.gyros_x_data = uchartoshort(receive_data_.buffer[14], receive_data_.buffer[15]);
                    receive_data_.IMUData.gyros_y_data = uchartoshort(receive_data_.buffer[16], receive_data_.buffer[17]);
                    receive_data_.IMUData.gyros_z_data = uchartoshort(receive_data_.buffer[18], receive_data_.buffer[19]);
                    receive_data_.Power_Voltage = (float)uchartoshort(receive_data_.buffer[20], receive_data_.buffer[21]) / 1000.0f;
                    receive_data_.Frame_Tail = receive_data_.buffer[23];

                    // 处理串口数据
                    //  将串口接收到的速度数据赋值给robot_vel_
                    robot_vel_.X = receive_data_.X_speed;
                    robot_vel_.Y = receive_data_.Y_speed;
                    robot_vel_.Z = receive_data_.Z_speed;

                    // 将串口接收到的IMU数据赋值给imu_msg_
                    imu_msg_.linear_acceleration.x = (float)receive_data_.IMUData.accele_x_data / ACCEl_RATIO;
                    imu_msg_.linear_acceleration.y = (float)receive_data_.IMUData.accele_y_data / ACCEl_RATIO;
                    imu_msg_.linear_acceleration.z = (float)receive_data_.IMUData.accele_z_data / ACCEl_RATIO;
                    imu_msg_.angular_velocity.x = (float)receive_data_.IMUData.gyros_x_data / GYROSCOPE_RATIO;
                    imu_msg_.angular_velocity.y = (float)receive_data_.IMUData.gyros_y_data / GYROSCOPE_RATIO;
                    imu_msg_.angular_velocity.z = (float)receive_data_.IMUData.gyros_z_data / GYROSCOPE_RATIO;
                    return true;
                }
            }
        }
    }

    return false;
}

/**
 * @brief Check sum
 * @param data Data array
 * @param len Data length
 * @return uint8_t Check sum
 */
uint8_t TurnOn32Chassis::check_sum(uint8_t *data, uint8_t len)
{
    unsigned char check_sum = 0, k;

    for (k = 0; k < len; k++)
    {
        check_sum = check_sum ^ data[k]; // By bit or by bit //按位异或
    }

    // if (mode == 3)
    // {
    //     for (k = 0; k < Count_Number; k++)
    //     {
    //         check_sum = check_sum ^ Distance_Data.rx[k]; // By bit or by bit //按位异或
    //     }

    return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}

void TurnOn32Chassis::Publish_Odom()
{
    // Convert the Z-axis rotation Angle into a quaternion for expression
    // 把Z轴转角转换为四元数进行表达
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_pos_.Z);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    nav_msgs::msg::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = rclcpp::Node::now();
    ;
    odom.header.frame_id = odom_frame_id;     // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = robot_pos_.X; // Position //位置
    odom.pose.pose.position.y = robot_pos_.Y;
    odom.pose.pose.position.z = 0.0; // Planar robot: robot_pos_.Z stores yaw, not height //二维机器人：robot_pos_.Z 表示偏航角，不是高度
    odom.pose.pose.orientation = odom_quat; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id;      // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x = robot_vel_.X;  // Speed in the X direction //X方向速度
    odom.twist.twist.linear.y = robot_vel_.Y;  // Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = robot_vel_.Z; // Angular velocity around the Z axis //绕Z轴角速度

    // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if (robot_vel_.X == 0 && robot_vel_.Y == 0 && robot_vel_.Z == 0)
        // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
        // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
        memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
            memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
        // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
        // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
        memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
            memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    odom_publisher->publish(odom); // Pub odometer topic //发布里程计话题
}
/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
// void TurnOn32Chassis::Publish_Voltage()
// {
//     std_msgs::msg::Float32 voltage_msgs; // Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
//     static float Count_Voltage_Pub = 0;
//     if (Count_Voltage_Pub++ > 10)
//     {
//         Count_Voltage_Pub = 0;
//         voltage_msgs.data = Power_voltage;        // The power supply voltage is obtained //电源供电的电压获取
//         voltage_publisher->publish(voltage_msgs); // Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
//     }
// }

void TurnOn32Chassis::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub; // Instantiate IMU topic data //实例化IMU话题数据
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id;        // IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
                                                         // IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
    Imu_Data_Pub.orientation.x = imu_msg_.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
    Imu_Data_Pub.orientation.y = imu_msg_.orientation.y;
    Imu_Data_Pub.orientation.z = imu_msg_.orientation.z;
    Imu_Data_Pub.orientation.w = imu_msg_.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
    Imu_Data_Pub.angular_velocity.x = imu_msg_.angular_velocity.x; // Triaxial angular velocity //三轴角速度
    Imu_Data_Pub.angular_velocity.y = imu_msg_.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = imu_msg_.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
    Imu_Data_Pub.linear_acceleration.x = imu_msg_.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
    Imu_Data_Pub.linear_acceleration.y = imu_msg_.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = imu_msg_.linear_acceleration.z;
    imu_publisher->publish(Imu_Data_Pub); // Pub IMU topic //发布IMU话题
}

void TurnOn32Chassis::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short transition; // intermediate variable //中间变量
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    has_received_cmd_vel_ = true;
    watchdog_stop_sent_ = false;

    send_data_.buffer[0] = FRAME_HEADER; // frame head 0x7B //帧头0X7B
    send_data_.buffer[1] = 0;            // set aside //预留位
    send_data_.buffer[2] = 0;            // set aside //预留位

    // The target velocity of the X-axis of the robot
    // 机器人x轴的目标线速度
    transition = 0;
    transition = twist_aux->linear.x * 1000; // 将浮点数放大一千倍，简化传输
    send_data_.buffer[4] = transition;       // 取数据的低8位
    send_data_.buffer[3] = transition >> 8;  // 取数据的高8位

    // The target velocity of the Y-axis of the robot
    // 机器人y轴的目标线速度
    transition = 0;
    transition = twist_aux->linear.y * 1000;
    send_data_.buffer[6] = transition;
    send_data_.buffer[5] = transition >> 8;

    // The target angular velocity of the robot's Z axis
    // 机器人z轴的目标角速度
    transition = 0;
    transition = twist_aux->angular.z * 1000;
    send_data_.buffer[8] = transition;
    send_data_.buffer[7] = transition >> 8;

    send_data_.buffer[9] = check_sum(send_data_.buffer, 9); // For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    send_data_.buffer[10] = FRAME_TAIL;                     // frame tail 0x7D //帧尾0X7D
    try
    {
        STM32_Serial.write(send_data_.buffer, sizeof(send_data_.buffer)); // Sends data to the downloader via serial port //通过串口向下位机发送数据
        RCLCPP_INFO(this->get_logger(), "Sent data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    send_data_.buffer[0], send_data_.buffer[1], send_data_.buffer[2], send_data_.buffer[3], send_data_.buffer[4],
                    send_data_.buffer[5], send_data_.buffer[6], send_data_.buffer[7], send_data_.buffer[8], send_data_.buffer[9], send_data_.buffer[10]);
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port")); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }
}

bool TurnOn32Chassis::send_stop_command()
{
    if (!STM32_Serial.isOpen())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Cannot send watchdog stop: serial port is not open");
        return false;
    }

    uint8_t tx_data[11] = {
        0x7B, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0X7D};
    tx_data[9] = check_sum(tx_data, 9);

    try
    {
        const size_t written = STM32_Serial.write(tx_data, sizeof(tx_data));
        if (written != sizeof(tx_data))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Incomplete watchdog stop frame: wrote %zu/%zu bytes",
                                  written, sizeof(tx_data));
            return false;
        }
        return true;
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Unable to send watchdog stop through serial port: %s",
                              e.what());
        return false;
    }
}

void TurnOn32Chassis::enforce_cmd_vel_timeout()
{
    if (cmd_vel_timeout_ <= 0.0 ||
        !has_received_cmd_vel_ ||
        watchdog_stop_sent_)
    {
        return;
    }

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_cmd_vel_time_).count();
    if (elapsed < cmd_vel_timeout_)
    {
        return;
    }

    if (send_stop_command())
    {
        watchdog_stop_sent_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "cmd_vel timed out for %.3f s (limit %.3f s); zero velocity sent to STM32",
                    elapsed, cmd_vel_timeout_);
    }
}

void TurnOn32Chassis::run()
{
    RCLCPP_INFO(this->get_logger(), "TurnOn32Chassis");
    last_time_ = rclcpp::Node::now();

    while (rclcpp::ok())
    {
        try
        {
            current_time_ = rclcpp::Node::now();
            sample_time_ = (current_time_ - last_time_).seconds();
            // RCLCPP_INFO(this->get_logger(), "sample_time_: %f", sample_time_);
            // 采样频率
            if (TurnOn32Chassis::receive_row_serialdata() == true)
            {
                // RCLCPP_INFO(this->get_logger(), "SAMPLING_FREQ: %f", 1 / sample_time_);

                // 应用缩放系数
                robot_vel_.X = robot_vel_.X * odom_x_scale_;
                robot_vel_.Y = robot_vel_.Y * odom_y_scale;
                if (robot_vel_.Z >= 0)
                    robot_vel_.Z = robot_vel_.Z * odom_z_scale_positive;
                else
                    robot_vel_.Z = robot_vel_.Z * odom_z_scale_negative;

                robot_pos_.X += (robot_vel_.X * cos(robot_pos_.Z) - robot_vel_.Y * sin(robot_pos_.Z)) * sample_time_; // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
                robot_pos_.Y += (robot_vel_.X * sin(robot_pos_.Z) + robot_vel_.Y * cos(robot_pos_.Z)) * sample_time_; // Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
                robot_pos_.Z += robot_vel_.Z * sample_time_;                                                          // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad

                // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
                Quaternion_Solution(imu_msg_.angular_velocity.x, imu_msg_.angular_velocity.y, imu_msg_.angular_velocity.z,
                                    imu_msg_.linear_acceleration.x, imu_msg_.linear_acceleration.y, imu_msg_.linear_acceleration.z);

                Publish_Odom();      // Pub the speedometer topic //发布里程计话题
                Publish_ImuSensor(); // Pub the IMU topic //发布IMU话题
                // Publish_Voltage();   // Pub the topic of power supply voltage //发布电源电压话题

                last_time_ = current_time_; // Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
            }

            rclcpp::spin_some(this->get_node_base_interface());
            enforce_cmd_vel_timeout();
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR(this->get_logger(), "unexpectedly failed whith %s", e.what());
        }
    }
}

TurnOn32Chassis::~TurnOn32Chassis()
{
    RCLCPP_INFO(this->get_logger(), "TurnOn32Chassis::~TurnOn32Chassis");

    send_stop_command();
    if (STM32_Serial.isOpen())
    {
        STM32_Serial.close();
    }
}

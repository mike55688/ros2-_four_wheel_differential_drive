#include <functional>
#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;//命名空间
using std::placeholders::_1;
using std::placeholders::_2;
serial::Serial ros_ser;
#define to_rad  0.017453f  //角度转弧度
void initialize_motor();
float V_x,V_y,V_z;
float x_step=0.3,y_step=0.3,z_step=0.3;  //設定xyz軸起始加速度
uint8_t Flag_Mode=0;
uint8_t Flag_move=0;//行动指令

uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2,count_3;//计数器
uint8_t FLAG_0,FLAG_1,FLAG_2,FLAG_3,FLAG_4,FLAG_5;//计数器

uint8_t Flag_start=0;//下位机运行标志

float angle_A,angle_B,angle_C,angle_D;//发送到下位机的4个轮子的角度
float speed_A,speed_B,speed_C,speed_D;//发送到下位机的4个轮子的速度

float Data_US[12];//发送到下位机的数据数组
float Data_UR[22];//接收来自下位机的数据数组
uint16_t a,b;
char aa;
void send_data(void);//串口发送协议函数
void process_and_send_data(char num);
void receive_and_process_data(void);
void process_data_and_get_odom(void);
void check_and_stop_vehicle();

rclcpp::Time last_command_time;  // 用于记录最后一次命令的时间
bool speed_command_received = false;  // 用于标记是否接收到速度命令
float dfbljblvdflb;
/*###################################################################################*/  
    double x = 0.0;

    double y = 0.0;

    double th = 0.0;

    double vx = 0.0;

    double vy = 0.0;

    double vth = 0.0;
    rclcpp::Time current_time, last_time;
      
// /*###################################################################################*/ 
// class OdometryPublisher : public rclcpp::Node
// {
// public:
//     OdometryPublisher()
//         : Node("odometry_publisher_" + std::to_string(std::rand() % 1000)), th_(0.0), vx_(0.0), vy_(0.0), vth_(0.0)
//     {
//         std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "odom");
//         pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
//         tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

//         timer_ = this->create_wall_timer(50ms, std::bind(&OdometryPublisher::publish_odometry, this));

//         last_time_ = this->get_clock()->now();
//     }

// private:
//     void publish_odometry()
//     {
//         auto current_time = this->get_clock()->now();
//         double dt = (current_time - last_time_).seconds();

//         // 使用全域變數計算位置
//         double delta_x = (vx * std::cos(th) - vy * std::sin(th)) * dt;
//         double delta_y = (vx * std::sin(th) + vy * std::cos(th)) * dt;
//         double delta_th = vth * dt;

//         x += delta_x; // 更新全域變數 x
//         y += delta_y; // 更新全域變數 y
//         th += delta_th; // 更新全域變數 th

//         // 更新 TF
//         geometry_msgs::msg::TransformStamped odom_trans;
//         odom_trans.header.stamp = current_time;
//         odom_trans.header.frame_id = "odom";
//         odom_trans.child_frame_id = "base_link";

//         odom_trans.transform.translation.x = x;
//         odom_trans.transform.translation.y = y;
//         odom_trans.transform.translation.z = 0.0;

//         tf2::Quaternion q;
//         q.setRPY(0, 0, th);
//         odom_trans.transform.rotation.x = q.x();
//         odom_trans.transform.rotation.y = q.y();
//         odom_trans.transform.rotation.z = q.z();
//         odom_trans.transform.rotation.w = q.w();

//         tf_broadcaster_->sendTransform(odom_trans);

//         // 發布里程計數據
//         nav_msgs::msg::Odometry odom;
//         odom.header.stamp = current_time;
//         odom.header.frame_id = "odom";

//         odom.pose.pose.position.x = x;
//         odom.pose.pose.position.y = y;
//         odom.pose.pose.position.z = 0.0;
//         odom.pose.pose.orientation.x = q.x();
//         odom.pose.pose.orientation.y = q.y();
//         odom.pose.pose.orientation.z = q.z();
//         odom.pose.pose.orientation.w = q.w();

//         odom.child_frame_id = "base_link";
//         odom.twist.twist.linear.x = vx;
//         odom.twist.twist.linear.y = vy;
//         odom.twist.twist.angular.z = vth;

//         pub_odom_->publish(odom);

//         last_time_ = current_time;
//     }

//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     double th_; // 使用類內變數僅存儲角度
//     double vx_, vy_, vth_; // 臨時變數
//     rclcpp::Time last_time_;
// };
/*###################################################################################*/
class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
        : Node("odometry_publisher_" + std::to_string(std::rand() % 1000)), count_(0)
    {
        // 初始化話題發布器
        pub_odom_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("wheel_odom", 10);

        // 初始化 TF 發布器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 設置計時器
        timer_ = this->create_wall_timer(50ms, std::bind(&OdometryPublisher::publish_odometry, this));
    }

private:
    void publish_odometry()
    {
        auto current_time = this->get_clock()->now();

        // 創建 TransformStamped 消息
        auto odom_translation = geometry_msgs::msg::TransformStamped();

        // 設置時間戳和框架資訊
        odom_translation.header.stamp = current_time;
        odom_translation.header.frame_id = "wheel_odom";
        odom_translation.child_frame_id = "base_link";

        // 填寫平移部分
        odom_translation.transform.translation.x = x;
        odom_translation.transform.translation.y = y;
        odom_translation.transform.translation.z = 0.0;

        // 填寫旋轉部分（四元數）
        tf2::Quaternion q;
        q.setRPY(0, 0, th);  // 假設只有 Z 軸旋轉
        odom_translation.transform.rotation.x = q.x();
        odom_translation.transform.rotation.y = q.y();
        odom_translation.transform.rotation.z = q.z();
        odom_translation.transform.rotation.w = q.w();

        // 發布 TransformStamped 到話題
        pub_odom_->publish(odom_translation);

        // 使用 TF 廣播器發布到 TF 树
        tf_broadcaster_->sendTransform(odom_translation);
    }

    // 私有成員變數
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_odom_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_; // 計數變數（目前未使用）
};

/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/ 

class Pospublisher : public rclcpp::Node  //發布機器人tf座標括位置 (x, y) 和方向 (theta)。此節點可用於模擬環境中監控機器人的實時位置和方向
{
public:
  Pospublisher(): Node("pos_publisher_" + std::to_string(std::rand() % 1000)), count_(0)
  {
    publisher_ = this->create_publisher<turtlesim::msg::Pose>("/pose", 2);
    timer_ = this->create_wall_timer(25ms, std::bind(&Pospublisher::timer_callback, this));//40HZ
  }
private:
  void timer_callback()
  {
    auto message = turtlesim::msg::Pose(); 
                                                                           
       message.x = x;
       message.y = y;
       message.theta = th; 
           
    publisher_->publish(message);               
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
  size_t count_;
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/ 
//原版
// class Velpublisher : public rclcpp::Node
// {
// public:
//   Velpublisher(): Node("vel_publisher_" + std::to_string(std::rand() % 1000)), count_(0)
//   {
//     publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 2);
//     timer_ = this->create_wall_timer(25ms, std::bind(&Velpublisher::timer_callback, this));//40HZ
//   }
// private:
//   void timer_callback()
//   {
//     auto message = geometry_msgs::msg::Twist();                                                              

//        message.linear.x = vx;
//        message.linear.y = vy;
//        message.angular.z = vth; 
           
//     publisher_->publish(message);               
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
//   size_t count_;
// };

/*###################################################################################*/ 
/*###################################################################################*/ 
/*###################################################################################*/ 
class Velpublisher : public rclcpp::Node
{
public:
    Velpublisher()
        : Node("vel_publisher_" + std::to_string(std::rand() % 1000)), vx(0.0), vy(0.0), vth(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 2);
        timer_ = this->create_wall_timer(10ms, std::bind(&Velpublisher::timer_callback, this));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Velpublisher::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();

        message.linear.x = vx;
        message.linear.y = vy;
        message.angular.z = vth;

        publisher_->publish(message);

        // 调试输出
        // std::cout << "Timer Publish: vx=" << vx << ", vy=" << vy << ", vth=" << vth << std::endl;

        if (vx != 0.0 || vy != 0.0 || vth != 0.0) {
            Data_US[0] = 1;
            Data_US[1] = vx + vth;
            Data_US[2] = vx - vth;
            Data_US[3] = vx - vth;
            Data_US[4] = vx + vth;

            send_data();
        }
    }
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vth = msg->angular.z;

    // 當速度全為 0，立即停止車輛
    if (vx == 0.0 && vy == 0.0 && vth == 0.0) {
        speed_A = 0.0;
        speed_B = 0.0;
        speed_C = 0.0;
        speed_D = 0.0;

        Data_US[0] = 1;  // 確保啟動電機
        Data_US[1] = 0.0;  // 停止速度
        Data_US[2] = 0.0;
        Data_US[3] = 0.0;
        Data_US[4] = 0.0;

        send_data();  // 發送停止指令
        std::cout << "[DEBUG] Stop command sent!" << std::endl;
    } else {
        // 正常處理速度
        Data_US[0] = 1;  // 確保啟動電機
        Data_US[1] = vx + vth;
        Data_US[2] = vx - vth;
        Data_US[3] = vx - vth;
        Data_US[4] = vx + vth;

        send_data();  // 發送速度指令
    }
}


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double vx, vy, vth;
};

/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_subscriber_" + std::to_string(std::rand() % 1000))
  {
       subscription_ = this->create_subscription<sensor_msgs::msg::Image>("Keyboard", 
       2, std::bind(&MinimalSubscriber::topic_callback, this, _1));    
  }
 
private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1) const
  {   
      aa = msg1->height;
  }                        
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 
};
 /*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // 初始化 ROS2

    // 配置串口
    ros_ser.setPort("/dev/ttyUSB0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ros_ser.setTimeout(to);

    try {
        ros_ser.open();
    } catch (serial::IOException &e) {
        std::cout << "Unable to open serial port" << std::endl;
        return -1;
    }

    if (!ros_ser.isOpen()) {
        std::cout << "Serial port not opened!" << std::endl;
        return -1;
    }
    std::cout << "/dev/ttyUSB0 is opened." << std::endl;

    // 初始化數據
    memset(Data_US, 0, sizeof(float) * 12);
    send_data();

    // 創建 ROS2 節點
    auto minimal_subscriber = std::make_shared<MinimalSubscriber>();
    auto pos_publisher = std::make_shared<Pospublisher>();
    auto vel_publisher = std::make_shared<Velpublisher>();
    auto odom_publisher = std::make_shared<OdometryPublisher>();

    // 使用 Executor 統一管理
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(minimal_subscriber);
    executor.add_node(pos_publisher);
    executor.add_node(vel_publisher);
    executor.add_node(odom_publisher);

    // 创建独立线程处理串口数据接收
    std::thread serial_thread([] {
        while (rclcpp::ok()) {
            receive_and_process_data();  // 独立线程处理串口数据接收
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 控制频率
        }
    });

    // 主线程运行 ROS 消息处理
    executor.spin();

    // 等待串口线程结束
    serial_thread.join();

    // 清理資源
    memset(Data_US, 0, sizeof(float) * 12);
    send_data();
    ros_ser.close();
    rclcpp::shutdown();
    return 0;
}
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
void send_data(void)
{
    uint8_t len = 12;
    unsigned char tbuf[53];
    unsigned char *p;

    for (uint8_t i = 0; i < len; i++) {
        p = (unsigned char *)&Data_US[i];
        tbuf[4 * i + 4] = (unsigned char)(*(p + 3));
        tbuf[4 * i + 5] = (unsigned char)(*(p + 2));
        tbuf[4 * i + 6] = (unsigned char)(*(p + 1));
        tbuf[4 * i + 7] = (unsigned char)(*(p + 0));
    }

    tbuf[len * 4 + 4] = 0;  // 校驗位
    tbuf[0] = 0XAA;         // 帧头
    tbuf[1] = 0XAA;         // 帧头
    tbuf[2] = 0XF1;         // 功能字
    tbuf[3] = len * 4;      // 数据长度
    for (uint8_t j = 0; j < (len * 4 + 4); j++) {
        tbuf[len * 4 + 4] += tbuf[j];  // 计算校验和
    }

    try {
        ros_ser.write(tbuf, len * 4 + 5);  // 發送數據
        std::cout << "[DEBUG] Data sent to lower board!" << std::endl;
    } catch (serial::IOException &e) {
        std::cout << "Unable to send data through serial port" << std::endl;
    }
}

//************************处理键盘的指令并发送数据到下位机**************************//
//************************处理键盘的指令并发送数据到下位机**************************// 
//************************处理键盘的指令并发送数据到下位机**************************//  
//************************处理键盘的指令并发送数据到下位机**************************//  
void process_and_send_data(char num)
{       
    rclcpp::Time now = rclcpp::Clock().now();  // 获取当前时间
 
    if(num=='u') Flag_move=1;//按键 u 左前
    else if(num=='i') Flag_move=2;//按键 i 前
    else if(num=='o') Flag_move=3;//按键 o 右前
    
    else if(num=='k') Flag_move=0;//按键 k 设置速度为0
    
    else if(num=='m') Flag_move=6;//按键 m 左后
    else if(num==',') Flag_move=7;//按键 (,)(<)后
    else if(num=='.') Flag_move=8;//按键 (.)(>)右后
    
    else if(num=='j') Flag_move=9;//按键 j 逆时针转
    else if(num=='l') Flag_move=10;//按键 l 顺时针转
    
    else if(num=='q') x_step+=0.05;//按键 q 某轴速度增加++
    else if(num=='z') x_step-=0.05;//按键 z 某轴速度减小--
    
    else if(num=='w') y_step+=0.05;//按键 w 某轴速度增加++
    else if(num=='x') y_step-=0.05;//按键 x 某轴速度减小--
    
    else if(num=='e') z_step+=0.05;//按键 e 某轴速度增加++
    else if(num=='c') z_step-=0.05;//按键 c 某轴速度减小--
    
    else {Flag_move=0;}//其余按键  

    if(x_step > +0.8)x_step = +0.8;
    if(x_step < -0.8)x_step = -0.8;
		
    if(y_step > +0.8)y_step = +0.8;
    if(y_step < -0.8)y_step = -0.8;

    if(z_step > +0.8)z_step = +0.8;
    if(z_step < -0.8)z_step = -0.8;		

    // 如果接收到有效速度指令，更新最后指令时间
    if (Flag_move != 0) {
        last_command_time = now;  // 记录最后一次指令时间
        speed_command_received = true;  // 设置为已接收到指令
    }

    if(Flag_move==2){//按下 I 键 //前进
        speed_A= x_step;
        speed_B= x_step; 
        speed_C= x_step; 
        speed_D= x_step; 
    }
    else if(Flag_move==7){//按下 < 键 //后退
        speed_A= -x_step;
        speed_B= -x_step; 
        speed_C= -x_step; 
        speed_D= -x_step;
    }
    else if(Flag_move==9){//按下 J 键//逆时针
        speed_A= +x_step;
        speed_B= -x_step; 
        speed_C= -x_step; 
        speed_D= +x_step; 
    }
    else if(Flag_move==10){//按下 L 键//顺时针
        speed_A= -x_step;
        speed_B= +x_step; 
        speed_C= +x_step; 
        speed_D= -x_step;  
    }
    else if(Flag_move==1){//按下 U 键//左斜上
        speed_A= x_step;
        speed_B= x_step*0.7F; 
        speed_C= x_step*0.5F; 
        speed_D= x_step*0.95F;  
    }
    else if(Flag_move==3){//按下 O 键//右斜上
        speed_B= x_step;
        speed_A= x_step*0.7F; 
        speed_D= x_step*0.5F; 
        speed_C= x_step*0.95F;
    }
    else if(Flag_move==6){//按下 M 键//左斜下
        speed_A= -x_step;
        speed_B= -x_step*0.7F; 
        speed_C= -x_step*0.5F; 
        speed_D= -x_step*0.95F;
    }
    else if(Flag_move==8){//按下 > 键//右斜下
        speed_B= -x_step;
        speed_A= -x_step*0.7F; 
        speed_D= -x_step*0.5F; 
        speed_C= -x_step*0.95F; 
    }
    else if(Flag_move==0){	       
        speed_A= 0;
        speed_B= 0; 
        speed_C= 0; 
        speed_D= 0; // 停止運動
    }

    /*<01>*/Data_US[0]  = 1; // 默認啟動電機
    /*<02>*/Data_US[1]  = speed_A; 
    /*<03>*/Data_US[2]  = speed_B ; 
    /*<04>*/Data_US[3]  = speed_C ; 
    /*<05>*/Data_US[4]  = speed_D ; // ABCD四轮的当前线速度 m/s
    /*<06>*/Data_US[5]  = 0 ;
    /*<07>*/Data_US[6]  = 0 ;    
    /*<08>*/Data_US[7]  = 0 ;    
    /*<09>*/Data_US[8]  = 0 ; 
    /*<10>*/Data_US[9]  = 0 ;//预留位  
    /*<11>*/Data_US[10] = 0 ;//预留位 
    /*<12>*/Data_US[11] = 0 ;//预留位

    if(++FLAG_1 == 2){
        FLAG_1 = 0;
        send_data(); //发送指令控制电机运行               
    }
}



void check_and_stop_vehicle() {
    rclcpp::Time now = rclcpp::Clock().now();  // 获取当前时间

    // 如果接收到指令，且距离上次指令超过 3 秒，则停止车辆
    if (speed_command_received) {
        double elapsed_time = (now - last_command_time).seconds();  // 计算时间间隔
        std::cout << "[DEBUG] Elapsed time since last command: " << elapsed_time << " seconds" << std::endl;

        if (elapsed_time > 3.0) {
            // 超时，停止车辆
            Flag_move = 0;  // 设置为停止状态
            speed_A = 0;
            speed_B = 0;
            speed_C = 0;
            speed_D = 0;

            // 发送停止命令
            send_data();
            std::cout << "[DEBUG] Sending stop command due to timeout." << std::endl;

            // 重置标志
            speed_command_received = false;
        }
    }
}


//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3)//float 型解算为4个字节
{if ((m0 == 0x00 || m0 == 0x80) && m1 == 0x00 && m2 == 0x00 && m3 == 0x00) return 0;
//求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//求阶码
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

void receive_and_process_data(void)
{        
//连续获取下位机的数据
	    /*<00>*///Data_UR[0] ;//电机启动开关，1启动 0停止
			/*<01>*///Data_UR[1] ;//X轴角速度原始数值 
			/*<02>*///Data_UR[2] ;//Y轴角速度原始数值 
			/*<03>*///Data_UR[3] ;//Z轴角速度原始数值
			/*<04>*///Data_UR[4] ;X轴加速度原始数值
			/*<05>*///Data_UR[5] ;Y轴加速度原始数值
			/*<06>*///Data_UR[6] ;Z轴加速度原始数值    
			/*<07>*///Data_UR[7] ;Z轴角度    
			/*<08>*///Data_UR[8] ;A轮速度
			/*<09>*///Data_UR[9] ;B轮速度 
			/*<10>*///Data_UR[10];C轮速度 
			/*<11>*///Data_UR[11];D轮速度
			/*<12>*///Data_UR[12];电压值*100 
			/*<13>*///Data_UR[13]; //预留
			/*<14>*///Data_UR[14]; //预留
						
  //获取下位机的数据				
               size_t n = ros_ser.available();//获取缓冲区内的字节数
            a++;
         if(n>0)
               {		   
                 uint8_t buffer[64];uint8_t buf[64];
                 
                 if(n>=130){
                   while(n){n = ros_ser.available();if(n>=130)ros_ser.read(buf, 62);else {break;}}//砍掉旧缓存，获取最新数据                   
                 }                 
                 if(n>=65 && n<130){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)ros_ser.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   ros_ser.read(buffer, 64);//读出64个字节                   
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<63;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[63] == (uint8_t)sum+buffer[0])
                          {b++;	
                           for(uint8_t i=0;i<15;i++)//15个数据
                            {
			     Data_UR[i] = b2f( buffer[4*i+3], buffer[4*i+4], buffer[4*i+5], buffer[4*i+6] );
                            }	                       										 				              				
	                   } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff; 
                   }
		     
                }    

        if(++FLAG_5>4){//显示频率降低
           FLAG_5=0;
           std::cout<<std::fixed<<std::setprecision(3)<< "[-3] V_x:" << vx <<"[m/s]" <<std::endl;//X轴速度 m/s
           std::cout<< "[-2] V_y:" << vy <<"[m/s]" <<std::endl;//Y轴速度 m/s
           std::cout<< "[-1] V_z:" << vth <<"[rad/s]" <<std::endl;//Z轴角速度 rad/s										
										
           if((uint16_t)Data_UR[0]==1){
            std::cout<< "[00] Flag_start:" << (uint16_t)Data_UR[0] <<std::endl;
            std::cout<< "[00] Flag_start: ON" <<std::endl;
            }//下位机电机启动/停止标志，1启动，0停止
           if((uint16_t)Data_UR[0]==0){
              std::cout<< "[00] Flag_start:" << (uint16_t)Data_UR[0] <<std::endl;
              std::cout<< "[00] Flag_start: OFF" <<std::endl;
             }//下位机电机启动/停止标志，1启动，0停止
                        
           std::cout<< "[01] gyro_Roll:" << (short)Data_UR[1] <<std::endl;//X轴角速度原始数据 gyro_Roll
           std::cout<< "[02] gyro_Pitch:" << (short)Data_UR[2] <<std::endl;//Y轴角速度原始数据 gyro_Pitch      
           std::cout<< "[03] gyro_Yaw:" << (short)Data_UR[3] <<std::endl;//z轴角速度原始数据 gyro_Yaw                                         
                      
           std::cout<< "[04] accel_x:" << (short)Data_UR[4] <<std::endl;//X轴加速度原始数据 accel_X
           std::cout<< "[05] accel_y:" << (short)Data_UR[5] <<std::endl;//Y轴加速度原始数据 accel_Y
           std::cout<< "[06] accel_z:" << (short)Data_UR[6] <<std::endl;//Z轴加速度原始数据 accel_Z  
                      								
           std::cout<<std::fixed<<std::setprecision(2)<<  "[07] Yaw:" << Data_UR[7] <<"[deg]"<<std::endl;//Z轴角度 deg 
						 
           std::cout<< "[08] Current_linear_A:" << +Data_UR[8] <<std::endl;//A轮线速度 m/s
           std::cout<< "[09] Current_linear_B:" << -Data_UR[9] <<std::endl;//B轮线速度 m/s
           std::cout<< "[10] Current_linear_C:" << -Data_UR[10] <<std::endl;//C轮线速度 m/s
           std::cout<< "[11] Current_linear_D:" << +Data_UR[11] <<std::endl;//D轮线速度 m/s						 
						 
           std::cout<< "[12] Voltage:" << Data_UR[12]/100 <<std::endl;//电池电压
				                       								 
           std::cout<< "[20] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[21] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[22] a/b:" <<  (float)a/b <<std::endl;
           std::cout<< "test line x : "<<(float)dfbljblvdflb<<std::endl;
           if(b>5000)b=b/10,a=a/10;
        
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}
//************************关于里程计**************************// 
//************************关于里程计**************************// 
//************************关于里程计**************************// 
void process_data_and_get_odom(void){
  
   double angular_velocity_x = Data_UR[1]*0.001064;//角速度转换成 rad/s
	 double angular_velocity_y = Data_UR[2]*0.001064;//角速度转换成 rad/s
	 double angular_velocity_z = Data_UR[3]*0.001064;//角速度转换成 rad/s			 
	 double accelerated_speed_x = Data_UR[4]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 double accelerated_speed_y = Data_UR[5]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 double accelerated_speed_z = Data_UR[6]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
		 
		 
	 //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负
	 //轮子从上往下看，逆时针转为正角度，顺时针转为负角度
         double Power_A_X =	+Data_UR[8];    //A轮X方向速度
         double Power_A_Y =	0;    //A轮Y方向速度
		
         double Power_B_X =	-Data_UR[9];    //A轮X方向速度
         double Power_B_Y =	0;    //A轮Y方向速度
		
         double Power_C_X =	-Data_UR[10];    //A轮X方向速度
         double Power_C_Y =	0;    //A轮Y方向速度
		
         double Power_D_X =	+Data_UR[11];    //A轮X方向速度
         double Power_D_Y =	0;    //A轮Y方向速度	
    	
    
          vx  = (Power_A_X + Power_B_X + Power_C_X + Power_D_X)/4 ;//底盘当前X方向线速度 m/s	
          vy  = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y)/4 ;//底盘当前Y方向线速度 m/s	
          vth = angular_velocity_z;//设备当前Z轴角速度 rad/s

          current_time = rclcpp::Clock().now();//记录当前时间
		
          //以给定机器人速度的典型方式计算里程计
          double dt = (current_time - last_time).seconds();//获得时间差并且单位转为秒

          double delta_x = (vx * cos(th) - vy * sin(th)) * dt;

          double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

          double delta_th = vth * dt;

 

            x += delta_x;//X轴速度累积位移 m

            y += delta_y;//Y轴速度累积位移 m

            th += delta_th;//Z轴角速度累积求车体朝向角度  rad //存在漂移
            			

}







void stop_vehicle_immediately()
{
    // 清零速度
    speed_A = 0.0;
    speed_B = 0.0;
    speed_C = 0.0;
    speed_D = 0.0;

    // 清空指令數據
    Data_US[0] = 1;  // 確保啟動電機
    Data_US[1] = 0.0;
    Data_US[2] = 0.0;
    Data_US[3] = 0.0;
    Data_US[4] = 0.0;

    send_data();  // 發送停止指令
    std::cout << "[DEBUG] Vehicle stopped immediately." << std::endl;
}

























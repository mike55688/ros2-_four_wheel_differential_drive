#include <functional>
#include <memory>
#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;
using std::placeholders::_2;
uint16_t count=0;
uint8_t flag=0;
double linear_x=0;
double linear_y=0;
double angular_z=0;
double position_x=0;
double position_y=0;
double position_z=0;
double translation_x;
double translation_y;
double translation_z;
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class TF2Subscriber : public rclcpp::Node
{
public:
  TF2Subscriber(): Node("TF2_Subscriber")
  {
       subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/odom", 
       2, std::bind(&TF2Subscriber::topic_callback, this, _1));         
  }
private:
  void topic_callback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr odom_translation) const
  {  
       translation_x = odom_translation->transform.translation.x;
       translation_y = odom_translation->transform.translation.y;
       translation_z = odom_translation->transform.translation.z;                            
  }                        
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_; 
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class VelSubscriber : public rclcpp::Node
{
public:
  VelSubscriber(): Node("Vel_Subscriber")
  {
       subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 
       2, std::bind(&VelSubscriber::topic_callback, this, _1));         
  } 
private:
  void topic_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg1) const
  {  
       linear_x =msg1->linear.x;
       linear_y =msg1->linear.y;
       angular_z =msg1->angular.z;                      
  }                        
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_; 
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class PosSubscriber : public rclcpp::Node
{
public:
  PosSubscriber(): Node("Pos_Subscriber")
  {
       subscription_ = this->create_subscription<turtlesim::msg::Pose>("/pose", 
       2, std::bind(&PosSubscriber::topic_callback, this, _1));         
  }
private:
  void topic_callback(const turtlesim::msg::Pose::ConstSharedPtr msg2) const
  {  
       position_x =msg2->x;
       position_y =msg2->y;
       position_z =msg2->theta;                      
  }                        
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_; 
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);//初始化 ROS2 客户端
  std::cout<< "**************"  <<std::endl;
  
  auto node1=std::make_shared<VelSubscriber>();
  auto node2=std::make_shared<PosSubscriber>();  
  auto node3=std::make_shared<TF2Subscriber>();
  rclcpp::Rate loop_rate(50);//设置循环间隔，即代码执行频率 50 HZ
  while(rclcpp::ok()){ 
    rclcpp::spin_some(node1);//调用spin_some函数，并传入节点对象指针  
    rclcpp::spin_some(node2);//调用spin_some函数，并传入节点对象指针    
    rclcpp::spin_some(node3);//调用spin_some函数，并传入节点对象指针
    if(++flag==2){flag=0;
      if(linear_x>=0)printf("[01]linear_x:+%.3f\n",linear_x);else printf("[01]linear_x:%.3f\n",linear_x);
      if(linear_y>=0)printf("[02]linear_y:+%.3f\n",linear_y);else printf("[02]linear_y:%.3f\n",linear_y);
      if(angular_z>=0)printf("[03]angular_z:+%.3f\n",angular_z);else printf("[03]angular_z:%.3f\n",angular_z);
      if(position_x>=0)printf("[04]position_x:+%.3f\n",position_x);else printf("[04]position_x:%.3f\n",position_x);
      if(position_y>=0)printf("[05]position_y:+%.3f\n",position_y);else printf("[05]position_y:%.3f\n",position_y);
      if(position_z>=0)printf("[06]position_z:+%.3f\n",position_z);else printf("[06]position_z:%.3f\n",position_z);
      if(translation_x>=0)printf("[07]translation_x:+%.3f\n",translation_x);else printf("[07]translation_x:%.3f\n",translation_x);
      if(translation_y>=0)printf("[08]translation_y:+%.3f\n",translation_y);else printf("[08]translation_y:%.3f\n",translation_y);  
      if(translation_z>=0)printf("[09]translation_z:+%.3f\n",translation_z);else printf("[09]translation_z:%.3f\n",translation_z);                   
        
      printf("[10]cycle:%d\n",count);
      printf("************************************************************\n"); 
    }  
    count++;
    if(count==65535)count=0;           
    loop_rate.sleep();//循环延时时间             
  }
  rclcpp::shutdown();//释放资源； 
  return 0;
}























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
#include "custom_msgs/msg/cmd_cut_pliers.hpp"

using namespace std::chrono_literals;//命名空间
using std::placeholders::_1;
using std::placeholders::_2;
serial::Serial ros_ser;
#define to_rad  0.017453f  //角度转弧度
 

uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2;//计数器
uint8_t Flag_start;
uint8_t Flag_OK=0;

int size;
int Voltage;

char aa;
uint16_t a,b;
void send_data(void);//串口发送协议函数
void process_and_send_data(char num);
void receive_and_process_data(void);
void initialize_arms();
int32_t S_H1;
int32_t S_L1;
int32_t S_H2;
int32_t S_L2;
uint8_t S_En1;
uint8_t S_En2;
uint8_t S_En3;
uint8_t S_En4;
uint8_t S_C1;
uint8_t S_C2;

int32_t R_H1;
int32_t R_L1;
int32_t R_H2;
int32_t R_L2;
uint8_t R_En1;
uint8_t R_En2;
uint8_t R_En3;
uint8_t R_En4;
uint8_t R_C1;
uint8_t R_C2;

/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class CmdCutPliersPublisher : public rclcpp::Node
{
public:
  CmdCutPliersPublisher()
      : Node("cmd_cut_pliers_publisher_" + std::to_string(std::rand() % 1000)),
        count_(0), height1(0), length1(0), height2(0), length2(0),
        target_height1(0), target_length1(0), target_height2(0), target_length2(0),
        claw1(false), claw2(false), step_size(10)
  {
    pub_cmd_cut_pliers_ = this->create_publisher<custom_msgs::msg::CmdCutPliers>("/cmd_cut_pliers", 10);

    sub_cmd_cut_pliers_ = this->create_subscription<custom_msgs::msg::CmdCutPliers>(
        "/cmd_cut_pliers", 10, std::bind(&CmdCutPliersPublisher::cmd_cut_pliers_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&CmdCutPliersPublisher::timer_callback, this));
  }

private:
  void cmd_cut_pliers_callback(const custom_msgs::msg::CmdCutPliers::SharedPtr msg) {
    target_height1 = msg->height1;
    target_length1 = msg->length1;
    target_height2 = msg->height2;
    target_length2 = msg->length2;
    S_C1 = msg->claw1;
    S_C2 = msg->claw2;

    S_En1 = msg->enable_motor1;
    S_En2 = msg->enable_motor2;
    S_En3 = msg->enable_motor3;
    S_En4 = msg->enable_motor4;
  }

  void timer_callback()
  { 
    auto msg = custom_msgs::msg::CmdCutPliers();

    if (S_H1 < target_height1) S_H1 += std::min(step_size, target_height1 - S_H1);
    else if (S_H1 > target_height1) S_H1 -= std::min(step_size, S_H1 - target_height1);

    if (S_L1 < target_length1) S_L1 += std::min(step_size, target_length1 - S_L1);
    else if (S_L1 > target_length1) S_L1 -= std::min(step_size, S_L1 - target_length1);

    if (S_H2 < target_height2) S_H2 += std::min(step_size, target_height2 - S_H2);
    else if (S_H2 > target_height2) S_H2 -= std::min(step_size, S_H2 - target_height2);

    if (S_L2 < target_length2) S_L2 += std::min(step_size, target_length2 - S_L2);
    else if (S_L2 > target_length2) S_L2 -= std::min(step_size, S_L2 - target_length2);

    if (S_H1 < 0) S_H1 = 0;
    if (S_H1 > 280) S_H1 = 280;
    if (S_L1 < 0) S_L1 = 0;
    if (S_L1 > 440) S_L1 = 440;
    if (S_H2 < 0) S_H2 = 0;
    if (S_H2 > 280) S_H2 = 280;
    if (S_L2 < 0) S_L2 = 0;
    if (S_L2 > 440) S_L2 = 440;

    msg.height1 = S_H1;
    msg.length1 = S_L1;
    msg.height2 = S_H2;
    msg.length2 = S_L2;
    msg.claw1 = S_C1;
    msg.claw2 = S_C2;

    pub_cmd_cut_pliers_->publish(msg);
    send_data();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::CmdCutPliers>::SharedPtr pub_cmd_cut_pliers_;
  rclcpp::Subscription<custom_msgs::msg::CmdCutPliers>::SharedPtr sub_cmd_cut_pliers_;

  int height1, length1, height2, length2;
  int target_height1, target_length1, target_height2, target_length2;
  int step_size;
  bool claw1, claw2;
  size_t count_;
};


/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_sync_subscriber")
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
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);//初始化 ROS2 客户端
/*
  auto t1 = std::chrono::system_clock::now();// 单位秒
  time_t tt = std::chrono::system_clock::to_time_t ( t1 );
  std::cout<< "Time:" << tt <<std::endl;  
*/ 
/*
  auto t = rclcpp::Clock().now();// 单位秒,纳秒
  std::cout<< "Time:" << t.seconds() << t.nanoseconds() <<std::endl; 
*/ 
        
  ros_ser.setPort("/dev/ttyUSB0");//端口号
  ros_ser.setBaudrate(115200);//波特率
  serial::Timeout to =serial::Timeout::simpleTimeout(100);//超时判定
  ros_ser.setTimeout(to);
  try
  {
    ros_ser.open();
  }
  catch(serial::IOException &e)
  {
    std::cout<<"unable to open"<<std::endl;
    return -1;
  }
  if(ros_ser.isOpen())
  {
    std::cout<<"/dev/ttyUSB0 is opened."<<std::endl;
  }
  else
  {
    return -1;
  }
  
      // 在這裡調用初始化函數
    initialize_arms();
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/   
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/ 
  auto minimal_subscriber = std::make_shared<MinimalSubscriber>();
  auto cmd_cut_pliers_publisher = std::make_shared<CmdCutPliersPublisher>();

      rclcpp::Rate loop_rate(150);//设置循环间隔，即代码执行频率 150 HZ,
      while(rclcpp::ok()){
           receive_and_process_data(); //接收并处理来自下位机的数据     
                       
				    // 調用 spin_some 函數處理 ROS 回調
           rclcpp::spin_some(minimal_subscriber);
           rclcpp::spin_some(cmd_cut_pliers_publisher);
          //  rclcpp::spin(cmd_cut_pliers_publisher);

           process_and_send_data(aa);//处理键盘的指令并发送数据到下位机            


         loop_rate.sleep();//循环延时时间             
      }
      
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/     

       S_En1=0;//电机1使能(控制手臂1的高度)
       S_En2=0;//电机2使能(控制手臂1的长度)
       S_En3=0;//电机3使能(控制手臂2的高度)
       S_En4=0;//电机4使能(控制手臂2的长度)
       S_C1=0;//爪子1 开/合 0/1
       S_C2=0;//爪子2 开/合 0/1
  send_data();
  ros_ser.close();//
  rclcpp::shutdown();//释放资源； 
  return 0;
}
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
void send_data(void)
{
    uint8_t tbuf[27];

    tbuf[26]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=22;    //数据长度
              tbuf[4]=	S_H1>>0;// 
              tbuf[5]=	S_H1>>8;//
              tbuf[6]=	S_H1>>16;//
              tbuf[7]=	S_H1>>24;//

              tbuf[8]=	S_L1>>0;// 
              tbuf[9]=	S_L1>>8;//
              tbuf[10]=	S_L1>>16;//
              tbuf[11]=	S_L1>>24;//

              tbuf[12]=	S_H2>>0;// 
              tbuf[13]=	S_H2>>8;//
              tbuf[14]=	S_H2>>16;//
              tbuf[15]=	S_H2>>24;//

              tbuf[16]=	S_L2>>0;// 
              tbuf[17]=	S_L2>>8;//
              tbuf[18]=	S_L2>>16;//
              tbuf[19]=	S_L2>>24;//
							
              tbuf[20]=	S_En1;//
              tbuf[21]=	S_En2;//
							tbuf[22]=	S_En3;//
              tbuf[23]=	S_En4;//
							
              tbuf[24]=	S_C1;//
              tbuf[25]=	S_C2;//
							
    for(uint8_t i=0;i<26;i++)tbuf[26]+=tbuf[i];//计算校验和 
  try{ros_ser.write(tbuf, 27);}//发送数据下位机(数组，字节数) 
  catch (serial::IOException& e){std::cout<<"Unable to send data through serial port"<<std::endl;}
  //如果发送数据失败，打印错误信息  
}  
//************************处理键盘的指令并发送数据到下位机**************************//
//************************处理键盘的指令并发送数据到下位机**************************// 
//************************处理键盘的指令并发送数据到下位机**************************//  
void process_and_send_data(char num)
{ 
static uint8_t FLAG_1=0;	
	   
         if(num=='h') Flag_start=1;//按键 h 
    else if(num=='g') Flag_start=0;//按键 g  
 
            
    aa=' ';//清除键盘回调值
		
	  //
		if(num=='q')S_H2+=10;//按键q上升
		else if(num=='z')S_H2-=10;//按键z下降
		if(num=='w')S_L2+=10;//按键w伸长		
		else if(num=='e')S_L2-=10;//按键e缩短
		
		if(num=='u')S_H1+=10;//按键u上升
		else if(num=='m')S_H1-=10;//按键m下降
		if(num=='i')S_L1+=10;//按键i伸长
		else if(num=='o')S_L1-=10;//按键o缩短	
	
		if(num==',')S_C1 =0;//按键u爪子张开
		else if(num=='.')S_C1 =1;//按键m爪子闭合
		
		if(num=='x')S_C2 =0;//按键i爪子张开
		else if(num=='c')S_C2 =1;//按键o爪子闭合	


	  if(S_H1<0)S_H1=0;
	  if(S_H1>280)S_H1=280;	
	  if(S_L1<0)S_L1=0;
	  if(S_L1>440)S_L1=440;
	  if(S_H2<0)S_H2=0;
	  if(S_H2>280)S_H2=280;	
	  if(S_L2<0)S_L2=0;
	  if(S_L2>440)S_L2=440;
		
    if(Flag_start==1){
					
			if(Flag_OK==0){
				 S_H1=0;//手臂1的高度 
         S_L1=0;//手臂1的长度
         S_H2=0;//手臂2的高度
         S_L2=0;//手臂2的长度
         S_En1=1;//电机1使能(控制手臂1的高度)
         S_En2=1;//电机2使能(控制手臂1的长度)
         S_En3=1;//电机3使能(控制手臂2的高度)
         S_En4=1;//电机4使能(控制手臂2的长度)
         S_C1=0;//爪子1 开/合 0/1
         S_C2=0;//爪子2 开/合 0/1				
				if(++FLAG_1<=2)send_data();
			}	else{	 
           if(++FLAG_1>=3){
              FLAG_1=0;
					    send_data(); //发送指令控制电机运行 
			     }
			}
	}
              
}
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 

void receive_and_process_data(void)
{        
//连续获取下位机的数据
			/*<01>*///buffer[4] ;//H1
			/*<02>*///buffer[5] ; 
			/*<03>*///buffer[6] ;
			/*<04>*///buffer[7] ; 
								
			/*<05>*///buffer[8] ;//L1
			/*<06>*///buffer[9] ;
			/*<07>*///buffer[10] ;    
			/*<08>*///buffer[11] ;  
								
			/*<09>*///buffer[12] ;//H2 
			/*<10>*///buffer[13] ; 
			/*<11>*///buffer[14];
			/*<12>*///buffer[15];
								
			/*<13>*///buffer[16];//L2 
			/*<14>*///buffer[17];
			/*<15>*///buffer[18];
      /*<16>*///buffer[19];	
								
      /*<17>*///buffer[20];//EN1
      /*<18>*///buffer[21];//EN2					 
			/*<19>*///buffer[22];//EN3	
			/*<20>*///buffer[23];//EN4
			
			/*<21>*///buffer[24];//C1
			/*<22>*///buffer[25];//C2

			/*<21>*///buffer[26];//Voltage
			/*<22>*///buffer[27];// 
			/*<21>*///buffer[28];// 
			/*<22>*///buffer[29];//
						
       //连续获取下位机的数据				
       size_t n = ros_ser.available();//获取缓冲区内的字节数
       a++;
         if(n>0)  
               {		   
                 uint8_t buffer[30];uint8_t buf[30];
                 
                 if(n>=62){
                   while(n){n = ros_ser.available();if(n>=62)ros_ser.read(buf, 30);else {break;}}//砍掉旧缓存，获取最新数据  ,预防堵塞                 
                 }                 
                 if(n>=31 && n<62){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)ros_ser.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   ros_ser.read(buffer, 30);//                 
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<29;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[29] == (uint8_t)(sum+buffer[0]))
                          {b++;	Flag_OK=1;
 						                   R_H1  = (int32_t)((buffer[3]<<0) |(buffer[4]<<8) |(buffer[5]<<16) |(buffer[6]<<24));//单位毫米
														   R_L1  = (int32_t)((buffer[7]<<0) |(buffer[8]<<8) |(buffer[9]<<16) |(buffer[10]<<24));//单位毫米
															 R_H2  = (int32_t)((buffer[11]<<0)|(buffer[12]<<8)|(buffer[13]<<16)|(buffer[14]<<24));//单位毫米
															 R_L2  = (int32_t)((buffer[15]<<0)|(buffer[16]<<8)|(buffer[17]<<16)|(buffer[18]<<24));//单位毫米
										
										           R_En1 = buffer[19];
										           R_En2 = buffer[20]; 
										           R_En3 = buffer[21]; 
										           R_En4 = buffer[22];
 										
						                   R_C1    = buffer[23]; 
														   R_C2    = buffer[24];
														
                               Voltage =(int32_t)((buffer[25]<<0) |(buffer[26]<<8) |(buffer[27]<<16) |(buffer[28]<<24));  	                       										 				              				
	                   } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff; 
                   }
		     
                } 
        if(++count_1>4){//显示频率降低
           count_1=0;
                       
           std::cout<< "[01] Current_Height_1:" << (int)R_H1 <<"[mm]"<<std::endl;
           std::cout<< "[02] Current_length_1:" << (int)R_L1 <<"[mm]"<<std::endl;

           std::cout<< "[03] Current_Height_2:" << (int)R_H2 <<"[mm]"<<std::endl;
           std::cout<< "[04] Current_length_2:" << (int)R_L2 <<"[mm]"<<std::endl;

           std::cout<< "[05] (Height_1)En1:" <<  (int)R_En1  <<std::endl;
           std::cout<< "[06] (length_1)En2:" <<  (int)R_En2  <<std::endl;

           std::cout<< "[07] (Height_2)En3:" <<   (int)R_En3  <<std::endl;
           std::cout<< "[08] (length_2)En4:" <<  (int)R_En4  <<std::endl;
					
           std::cout<< "[09] State_Claw1:" <<   (int)R_C1  <<std::endl;
           std::cout<< "[10] State_Claw2:" <<  (int)R_C2  <<std::endl;
					
           std::cout<< "[11] Voltage:" << (float)Voltage/100 <<std::endl;//电池电压
                      								 
           std::cout<< "[12] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[13] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[14] a/b:" <<  (float)a/b <<std::endl;
           if(b>5000)b=b/10,a=a/10;
         
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}



void initialize_arms()
{
    // 初始化所有手臂參數
    S_H1 = 0; // 手臂1的高度
    S_L1 = 0; // 手臂1的長度
    S_H2 = 0; // 手臂2的高度
    S_L2 = 0; // 手臂2的長度
    S_En1 = 1; // 啟用電機1
    S_En2 = 1; // 啟用電機2
    S_En3 = 1; // 啟用電機3
    S_En4 = 1; // 啟用電機4
    S_C1 = 0;  // 爪子1張開
    S_C2 = 0;  // 爪子2張開

    send_data(); // 發送初始化數據到下位機
    RCLCPP_INFO(rclcpp::get_logger("initialize_arms"), "Arms initialized with default values.");
}





























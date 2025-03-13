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

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
serial::Serial ros_ser;
#define to_rad  0.017453f  //角度转弧度

// 全局變數（已刪除與手臂2相關的部分）
uint8_t FLAG_USART; // 串口發送標誌
uint16_t count_1;   // 計數器（手臂1狀態顯示用）
uint8_t Flag_start;
uint8_t Flag_OK = 0;

int size;
int Voltage;

char aa;
uint16_t a, b;
void send_data(void);
void process_and_send_data(char num);
void receive_and_process_data(void);
void initialize_arms();

// 控制手臂1的目標位置與參數
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

/*###################################################################################
  CmdCutPliersPublisher 節點：訂閱 /cmd_cut_pliers topic 並發送數據到下位機
###################################################################################*/
// class CmdCutPliersPublisher : public rclcpp::Node
// {
// public:
//   CmdCutPliersPublisher()
//       : Node("cmd_cut_pliers_publisher_" + std::to_string(std::rand() % 1000)),
//         count_(0), height1(0), length1(0),
//         target_height1(0), target_length1(0),
//         claw1(false), step_size(10)
//   {
//     pub_cmd_cut_pliers_ = this->create_publisher<custom_msgs::msg::CmdCutPliers>("/cmd_cut_pliers", 10);

//     sub_cmd_cut_pliers_ = this->create_subscription<custom_msgs::msg::CmdCutPliers>(
//         "/cmd_cut_pliers", 10, std::bind(&CmdCutPliersPublisher::cmd_cut_pliers_callback, this, _1));

//     timer_ = this->create_wall_timer(100ms, std::bind(&CmdCutPliersPublisher::timer_callback, this));
//   }

// private:
//   // 當收到 /cmd_cut_pliers 訊息時，更新手臂1目標高度、長度、爪子狀態與使能狀態
//   void cmd_cut_pliers_callback(const custom_msgs::msg::CmdCutPliers::SharedPtr msg) {
//     target_height1 = msg->height1;
//     target_length1 = msg->length1;
//     S_C1 = msg->claw1;
//     S_En1 = 1;
//     S_En2 = 1;
//   }

//   // 定時回呼：邊界檢查後將數據封包並呼叫 send_data() 發送到下位機
//   void timer_callback() {
//     auto msg = custom_msgs::msg::CmdCutPliers();

//     // 一次到位：設定手臂1的目標高度與長度
//     S_H1 = target_height1;
//     S_L1 = target_length1;

//     // 邊界檢查
//     S_H1 = std::clamp(S_H1, 0, 280);
//     S_L1 = std::clamp(S_L1, 0, 440);

//     msg.height1 = S_H1;
//     msg.length1 = S_L1;
//     msg.claw1 = S_C1;

//     pub_cmd_cut_pliers_->publish(msg);
//     send_data();
//   }

//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<custom_msgs::msg::CmdCutPliers>::SharedPtr pub_cmd_cut_pliers_;
//   rclcpp::Subscription<custom_msgs::msg::CmdCutPliers>::SharedPtr sub_cmd_cut_pliers_;

//   int height1, length1;
//   int target_height1, target_length1;
//   int step_size;
//   bool claw1;
//   size_t count_;
// };

/*###################################################################################
  CmdCutPliersPublisher 節點：訂閱 /cmd_cut_pliers topic 並發送數據到下位機 OK
###################################################################################*/
class CmdCutPliersPublisher : public rclcpp::Node
{
public:
  CmdCutPliersPublisher()
      : Node("cmd_cut_pliers_publisher_" + std::to_string(std::rand() % 1000)),
        target_height1(140), target_length1(-1), last_valid_length_(10), claw1(false), allow_retract_(false)
  {
    clock_ = this->get_clock();  // ✅ 使用 ROS2 Clock
    sub_cmd_cut_pliers_ = this->create_subscription<custom_msgs::msg::CmdCutPliers>(
        "/cmd_cut_pliers", 1,
        std::bind(&CmdCutPliersPublisher::cmd_cut_pliers_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CmdCutPliersPublisher::timer_callback, this));
  }

private:
  int target_height1;
  int target_length1;
  int last_valid_length_;
  bool claw1;
  bool allow_retract_;
  rclcpp::Clock::SharedPtr clock_;  // ✅ 使用 ROS2 Clock

  rclcpp::Subscription<custom_msgs::msg::CmdCutPliers>::SharedPtr sub_cmd_cut_pliers_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cmd_cut_pliers_callback(const custom_msgs::msg::CmdCutPliers::SharedPtr msg)
  {
      bool updated = false;
  
      // 🚀 **前進模式**
      if (msg->mode == 0) {  
          if (!allow_retract_ && msg->length1 >= last_valid_length_) {  
              if (target_length1 < msg->length1) { // ✅ 確保 target_length1 只能變大
                  target_length1 = msg->length1;
                  last_valid_length_ = target_length1;
                  updated = true;
              } else {
                  RCLCPP_WARN(this->get_logger(), "⚠ 忽略前進指令，因為目標長度 (%d) 小於當前長度 (%d)", 
                              msg->length1, last_valid_length_);
              }
          }
      } 
      // 🔄 **後退模式**
      else if (msg->mode == 1) {  
          if (msg->length1 < last_valid_length_) {  // **只允許長度變小**
              target_length1 = msg->length1;
              last_valid_length_ = msg->length1;
              updated = true;
          } else {
              RCLCPP_WARN(this->get_logger(), "⚠ 忽略後退指令，因為目標長度 (%d) 比當前長度 (%d) 更大", 
                          msg->length1, last_valid_length_);
          }
      }
  
      // ✅ **更新高度**
      if (msg->height1 >= 0 && target_height1 != msg->height1) {
          target_height1 = msg->height1;
          updated = true;
      }
  
      // ✅ **更新剪鉗狀態**
      if (claw1 != msg->claw1) {
          claw1 = msg->claw1;
          updated = true;
      }
  
      if (updated) {
          RCLCPP_INFO(this->get_logger(), "✅ 目標更新: height=%d, length=%d, mode=%d", 
                      target_height1, target_length1, msg->mode);
      }
  }
  
    

  void timer_callback()
  {
    rclcpp::Time current_time = clock_->now();  // ✅ 統一時間來源

    if (allow_retract_) {
        S_L1 = std::max(target_length1, 10); // ✅ **允許縮短**
    } else {
        S_L1 = std::max(last_valid_length_, 10); // ❌ **確保不會影響縮回**
    }

    S_H1 = std::clamp(target_height1, 0, 280);
    S_C1 = claw1;

    RCLCPP_INFO(this->get_logger(), "📢 [%f] 發送數據: height=%d, length=%d, claw=%s",
                current_time.seconds(), S_H1, S_L1, S_C1 ? "True" : "False");

    send_data();
  }
};

//
/*###################################################################################
  ArmStatusPublisher 節點：發布手臂當前狀態到 /arm_current_status 主題
###################################################################################*/
class ArmStatusPublisher : public rclcpp::Node
{
public:
  ArmStatusPublisher()
      : Node("arm_status_publisher")
  {
    pub_arm_status_ = this->create_publisher<custom_msgs::msg::CmdCutPliers>("/arm_current_status", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&ArmStatusPublisher::timer_callback, this));
  }

private:
  // 在定時器回調中，使用全域變數 R_H1, R_L1, R_C1 發布手臂當前狀態
  void timer_callback() {
    auto msg = custom_msgs::msg::CmdCutPliers();
    msg.height1 = R_H1;   // 來自 receive_and_process_data() 更新的手臂高度
    msg.length1 = R_L1;   // 來自 receive_and_process_data() 更新的手臂長度
    msg.claw1 = S_C1;  // 根據 R_C1 判斷爪子狀態，非 0 為 true
    pub_arm_status_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing arm status: height=%d, length=%d, claw=%d", R_H1, R_L1, (R_C1 != 0));
  }

  rclcpp::Publisher<custom_msgs::msg::CmdCutPliers>::SharedPtr pub_arm_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};


/*###################################################################################
  MinimalSubscriber 節點：訂閱 "Keyboard" topic 接收鍵盤指令
###################################################################################*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_sync_subscriber")
  {
       subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
           "Keyboard", 2, std::bind(&MinimalSubscriber::topic_callback, this, _1));    
  }
 
private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1) const
  {   
      // 這裡利用 image 的 height 欄位暫存鍵盤輸入
      aa = msg1->height;
  }                        
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 
};

/*###################################################################################
  主函數
###################################################################################*/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // 初始化串口
  ros_ser.setPort("/dev/ttyUSB1");
  ros_ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  ros_ser.setTimeout(to);
  try {
    ros_ser.open();
  }
  catch(serial::IOException &e) {
    std::cerr << "Unable to open serial port" << std::endl;
    return -1;
  }
  if (ros_ser.isOpen()) {
    std::cout << "/dev/ttyUSB0 is opened." << std::endl;
  } else {
    return -1;
  }
  
  // 初始化手臂參數
  initialize_arms();

  // 建立各個節點物件
  auto minimal_subscriber = std::make_shared<MinimalSubscriber>();
  auto cmd_cut_pliers_publisher = std::make_shared<CmdCutPliersPublisher>();
  auto arm_status_publisher = std::make_shared<ArmStatusPublisher>();

  // 使用 MultiThreadedExecutor 同時執行多個節點
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(minimal_subscriber);
  executor.add_node(cmd_cut_pliers_publisher);
  executor.add_node(arm_status_publisher);

  // 啟動一個執行緒來持續 spin 這些節點
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // 主迴圈：每 20 毫秒處理一次下位機數據
  rclcpp::Rate loop_rate(150);
  while (rclcpp::ok()) {
    receive_and_process_data();  // 接收並處理下位機數據     
    // 若需要根據鍵盤指令處理並發送數據，可在此呼叫 process_and_send_data(...)
    // loop_rate.sleep();
  }
      
  // 結束前將手臂停止
  S_En1 = 0; // 停止高度控制
  S_En2 = 0; // 停止長度控制
  S_C1 = 0;  // 爪子張開（依需求設定）
  send_data();
  ros_ser.close();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

/*###################################################################################
  send_data() 函數：打包手臂1控制數據並透過串口發送
  修改後的數據帧結構：
    - 帧頭：2 bytes (0xAA, 0xAA)
    - 功能字：1 byte (0xF1)
    - 數據長度：1 byte (11，表示後續有11個字節)
    - S_H1：4 bytes (手臂1高度)
    - S_L1：4 bytes (手臂1長度)
    - S_En1：1 byte (電機1使能)
    - S_En2：1 byte (電機2使能)
    - S_C1：1 byte (爪子狀態)
    - 校驗和：1 byte (前15個字節的總和)
    → 總長度 16 bytes
###################################################################################*/
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
/*###################################################################################
  process_and_send_data() 函數：根據鍵盤輸入控制手臂1運動
    - 按鍵 'u'：手臂1上升 (高度增加)
    - 按鍵 'm'：手臂1下降 (高度減少)
    - 按鍵 'i'：手臂1伸長 (長度增加)
    - 按鍵 'o'：手臂1縮短 (長度減少)
    - 按鍵 ','：爪子張開 (S_C1 = 0)
    - 按鍵 '.'：爪子閉合 (S_C1 = 1)
    - 按鍵 'h'：開始動作 (Flag_start = 1)
    - 按鍵 'g'：停止動作 (Flag_start = 0)
###################################################################################*/
void process_and_send_data(char num)
{ 
    static uint8_t FLAG_1 = 0;	
	   
    if(num == 'h') Flag_start = 1; // 按鍵 h 開始
    else if(num == 'g') Flag_start = 0; // 按鍵 g 停止
 
    aa = ' '; // 清除鍵盤輸入值
		
    // 手臂1控制（不再處理手臂2相關指令）
    if(num == 'u') S_H1 += 10;   // 按鍵 u：上升
    else if(num == 'm') S_H1 -= 10; // 按鍵 m：下降
    if(num == 'i') S_L1 += 10;   // 按鍵 i：伸長
    else if(num == 'o') S_L1 -= 10; // 按鍵 o：縮短	
	
    if(num == ',') S_C1 = 0;     // 按鍵 ,：爪子張開
    else if(num == '.') S_C1 = 1; // 按鍵 .：爪子閉合
	
    if(S_H1 < 0) S_H1 = 0;
    if(S_H1 > 280) S_H1 = 280;	
    if(S_L1 < 0) S_L1 = 0;
    if(S_L1 > 440) S_L1 = 440;
		
    if(Flag_start == 1){
        if(Flag_OK == 0){
            // 初始化手臂1的數值
            S_H1 = 0;
            S_L1 = 0;
            S_En1 = 1;
            S_En2 = 1;
            S_C1 = 0;
            if(++FLAG_1 <= 2) send_data();
        } else {	 
            if(++FLAG_1 >= 3){
                FLAG_1 = 0;
                send_data(); // 發送指令控制手臂運動
            }
        }
    }            
}

/*###################################################################################
  receive_and_process_data() 函數：接收下位機傳回的手臂1狀態數據，並解析校驗
    修改後的數據帧結構總長 16 字節
###################################################################################*/
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

										           R_En1 = buffer[19];
										           R_En2 = buffer[20]; 

 										
						                   R_C1    = buffer[23]; 
														
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

           std::cout<< "[05] (Height_1)En1:" <<  (int)R_En1  <<std::endl;
           std::cout<< "[06] (length_1)En2:" <<  (int)R_En2  <<std::endl;

           std::cout<< "[09] State_Claw1:" <<   (int)R_C1  <<std::endl;
					
           std::cout<< "[11] Voltage:" << (float)Voltage/100 <<std::endl;//电池电压
                      								 
           std::cout<< "[12] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[13] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[14] a/b:" <<  (float)a/b <<std::endl;
           if(b>5000)b=b/10,a=a/10;
         
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}

/*###################################################################################
  initialize_arms() 函數：初始化手臂1參數並發送初始數據到下位機
###################################################################################*/
void initialize_arms()
{
    S_H1 = 0;  // 手臂1高度初始化
    S_L1 = 0;  // 手臂1長度初始化
    S_En1 = 1; // 啟用電機1
    S_En2 = 1; // 啟用電機2
    S_C1 = 0;  // 爪子1張開

    send_data(); // 發送初始化數據到下位機
}


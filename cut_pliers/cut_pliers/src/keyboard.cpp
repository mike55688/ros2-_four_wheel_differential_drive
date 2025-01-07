#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
 
using namespace std::chrono_literals;//命名空间
class KeyboardPublisher : public rclcpp::Node//使用面向对象中的继承，自定义类名为MinimalPublisher继承rclcpp::Node这个类，继承方式为public;
{
//公有成员:类的构造函数用于对私有成员进行初始化；
//公共构造函数命名节点minimal_publisher并初始化count_为 0。在构造函数内部，发布者使用Image消息类型、主题名称topic和所需的队列大小进行初始化，
//以在发生备份时限制消息。接下来，timer_被初始化，这会导致timer_callback函数每秒执行 次。this指的是该节点
public:
  KeyboardPublisher(): Node("Keyboard_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Keyboard", 2);//创建发布方
    timer_ = this->create_wall_timer(20ms, std::bind(&KeyboardPublisher::timer_callback, this));//执行频率50HZ，
  }
 //私有成员有回调函数timer_callback，计时器对象timer_,发布者对象publisher_,统计数量的count_;
 //该timer_callback函数是设置消息数据和实际发布消息的地方。该RCLCPP_INFO宏确保每个发布的消息都打印到控制台。还有计时器、发布者和计数器字段的声明
private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::Image();
     char s; 
    //std::cin >> s;
    RCLCPP_INFO(get_logger(), " ");
    system("stty -icanon");//关闭缓冲区，输入字符无需回车直接接受
    s = std::getchar();     
                 
    if(s=='u'||s=='i'||s=='o'||s=='j'||s=='k'||s=='l'||s=='m'||s==','
    ||s=='.'||s=='t'||s=='b'||s=='q'||s=='z'||s=='w'||s=='x'||s=='e'||s=='c'||s=='h'||s=='g') {}
    else { 
     std::cout << "请输入正确按键|请用小写健"<< 
     std::endl;}                                                                 
        
    message.height = s;   
    publisher_->publish(message);               
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};
//主函数部分
//rclcpp::init初始化 ROS 2，并rclcpp::spin开始处理来自节点的数据，包括来自计时器的回调。
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);//初始化 ROS2 客户端；
  std::cout<<"Keys:"<<std::endl;
  std::cout<<"Q  W  E  T     U  I  O  "<<std::endl;
  std::cout<<"          G  H  J  K  L  "<<std::endl; 
  std::cout<<" Z  X  C   B     M  ,  .  "<<std::endl;             
  rclcpp::spin(std::make_shared<KeyboardPublisher>());//调用spin函数，并传入节点对象指针。  
  rclcpp::shutdown();//释放资源；
  return 0;
}

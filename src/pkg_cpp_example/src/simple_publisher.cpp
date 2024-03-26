#include <rclcpp/rclcpp.hpp>  // 引入ROS2客户端库的核心头文件
#include <std_msgs/msg/string.hpp>  // 引入std_msgs中String消息类型的头文件

#include <chrono>  // 引入chrono库，用于处理时间相关的功能

using namespace std::chrono_literals;  // 允许使用后缀"s"来表示秒

// 定义一个SimplePublisher类，继承自rclcpp::Node
class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher"), counter_(0)  // 构造函数，初始化节点名称和计数器
  {
    // 创建一个发布者，发布std_msgs::msg::String类型的消息到"chatter"话题，消息队列长度为10
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    // 创建一个定时器，每秒调用一次timerCallback函数
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    // 输出日志信息，表示发布频率为1Hz
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }

  // 定时器回调函数，当定时器触发时调用
  void timerCallback()
  {
    auto message = std_msgs::msg::String();  // 创建一个String类型的消息
    // 设置消息内容为"Hello ROS 2 - counter:"加上计数器的值，并将计数器递增
    message.data = "Hello ROS 2 - counter:" + std::to_string(counter_++);
    // 发布消息
    pub_->publish(message);
  }

private:
  // 创建一个std_msgs::msg::String类型的发布者共享指针
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  // 创建一个定时器共享指针
  rclcpp::TimerBase::SharedPtr timer_;
  // 创建一个无符号整型计数器
  unsigned int counter_;
};

// main函数，程序的入口点
int main(int argc, char* argv[])
{
  // 初始化ROS2客户端库
  rclcpp::init(argc, argv);
  // 创建SimplePublisher节点的一个共享指针实例
  auto node = std::make_shared<SimplePublisher>();
  // 进入事件循环，等待回调函数执行
  rclcpp::spin(node);
  // 关闭ROS2客户端库
  rclcpp::shutdown();
  // 返回0，表示程序正常退出
  return 0;
}
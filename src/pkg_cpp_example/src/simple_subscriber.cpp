#include <rclcpp/rclcpp.hpp>       // 导入rclcpp库，这是C++的ROS2客户端库
#include <std_msgs/msg/string.hpp> // 导入std_msgs消息库中的String消息类型

using std::placeholders::_1; // 使用C++标准库中的placeholders命名空间，用于绑定回调函数

// 定义一个SimpleSubscriber类，继承自rclcpp::Node类
class SimpleSubscriber : public rclcpp::Node
{
public:
    // 构造函数
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // 创建一个订阅者，订阅名为"chatter"的话题，消息类型为std_msgs::msg::String，队列长度为10
        // 使用std::bind绑定回调函数msgCallback，_1代表传入回调函数的第一个参数（消息）
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
    }

private:
    // 定义一个共享指针，用于存储订阅者的句柄
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // 定义一个msgCallback方法，用于处理接收到的消息
    // const关键字表示这个方法不会修改类的成员变量
    void msgCallback(const std_msgs::msg::String &msg) const
    {
        // 使用RCLCPP_INFO_STREAM宏打印接收到的消息内容
        // this->get_logger()获取节点的日志记录器
        // msg.data.c_str()将接收到的字符串消息转换为C风格字符串
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
    }
};

// 定义一个main函数，作为程序的入口点
int main(int argc, char *argv[])
{
    // 初始化rclcpp库
    rclcpp::init(argc, argv);
    // 创建一个指向SimpleSubscriber类对象的新共享指针
    auto node = std::make_shared<SimpleSubscriber>();
    // 将node其传递给rclcpp::spin进入事件循环
    rclcpp::spin(node);
    // 关闭rclcpp库
    rclcpp::shutdown();
    // 程序正常退出
    return 0;
}

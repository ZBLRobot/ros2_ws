#include <rclcpp/rclcpp.hpp>                   // 导入rclcpp头文件，这是ROS2的C++客户端库
#include "pkg_interfaces/srv/add_two_ints.hpp" // 导入自定义服务消息头文件

#include <chrono> // 导入chrono头文件，用于处理时间相关的操作
#include <memory> // 导入memory头文件，用于智能指针

using namespace std::chrono_literals; // 使用chrono_literals命名空间，允许使用时间字面量，如1s
using std::placeholders::_1;          // 使用placeholders命名空间中的_1占位符，用于回调函数

// 定义一个名为SimpleServiceClient的类，继承自rclcpp::Node
class SimpleServiceClient : public rclcpp::Node
{
public:
    // 构造函数，接收两个整数参数
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        client_ = create_client<pkg_interfaces::srv::AddTwoInts>("add_two_ints"); // 创建一个客户端，用于调用add_two_ints服务

        auto request = std::make_shared<pkg_interfaces::srv::AddTwoInts::Request>(); // 创建一个请求对象
        request->a = a;                                                              // 设置请求对象的a字段
        request->b = b;                                                              // 设置请求对象的b字段

        // 等待服务服务器可用
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); // 如果等待过程中被中断，则打印错误信息并退出
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again..."); // 如果服务不可用，则每秒打印一次等待信息
        }

        // 发送异步请求，并绑定响应回调函数
        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

private:
    rclcpp::Client<pkg_interfaces::srv::AddTwoInts>::SharedPtr client_; // 声明一个服务客户端共享指针

    // 定义响应回调函数
    void responseCallback(rclcpp::Client<pkg_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service Response " << future.get()->sum); // 如果响应有效，则打印服务响应的结果
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Failure"); // 如果响应无效，则打印服务失败的信息
        }
    }
};

// 定义主函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化rclcpp

    // 检查命令行参数数量是否正确
    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments! Usage: simple_service_client A B");
        return 1;
    }

    // 创建SimpleServiceClient对象，传入命令行参数作为两个整数
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node); // 保持节点运行，等待服务响应
    rclcpp::shutdown(); // 关闭rclcpp
    return 0;           // 程序正常退出
}
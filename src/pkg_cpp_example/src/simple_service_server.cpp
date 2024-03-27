#include <rclcpp/rclcpp.hpp>                   // 导入rclcpp头文件，这是ROS2的C++客户端库
#include "pkg_interfaces/srv/add_two_ints.hpp" // 导入自定义服务消息头文件

#include <memory> // 导入内存头文件，用于智能指针

using namespace std::placeholders; // 导入 placeholders 命名空间，用于 std::bind

// 定义一个名为SimpleServiceServer的类，继承自rclcpp::Node
class SimpleServiceServer : public rclcpp::Node
{
public:
    // 构造函数
    SimpleServiceServer() : Node("simple_service_server")
    {
        // 创建一个服务，服务类型为pkg_interfaces::srv::AddTwoInts，服务名为add_two_ints，回调函数为serviceCallback
        service_ = create_service<pkg_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
        // 在日志中输出服务add_two_ints已准备好的信息
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints Ready");
    }

private:
    // 声明一个服务句柄
    rclcpp::Service<pkg_interfaces::srv::AddTwoInts>::SharedPtr service_;

    // 定义服务回调函数
    void serviceCallback(const std::shared_ptr<pkg_interfaces::srv::AddTwoInts::Request> req,
                         const std::shared_ptr<pkg_interfaces::srv::AddTwoInts::Response> res)
    {
        // 在日志中输出新的请求接收到的信息
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Request Received a: " << req->a << " b: " << req->b);
        // 计算两个整数的和，并赋值给响应对象的sum字段
        res->sum = req->a + req->b;
        // 在日志中输出返回的和
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning sum: " << res->sum);
    }
};

// 定义主函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                            // 初始化rclcpp
    auto node = std::make_shared<SimpleServiceServer>(); // 创建SimpleServiceServer对象
    rclcpp::spin(node);                                  // 保持节点运行，等待服务请求
    rclcpp::shutdown();                                  // 关闭rclcpp
    return 0;                                            // 程序正常退出
}
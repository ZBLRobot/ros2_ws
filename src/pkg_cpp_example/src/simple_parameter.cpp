#include <rclcpp/rclcpp.hpp>                            // 导入rclcpp头文件，这是ROS2的C++客户端库
#include <rcl_interfaces/msg/set_parameters_result.hpp> // 导入rcl_interfaces消息中set_parameters_result消息类型

#include <vector> // 导入vector头文件，用于存储参数列表
#include <string> // 导入string头文件，用于字符串操作
#include <memory> // 导入memory头文件，用于智能指针

using std::placeholders::_1; // 使用placeholders命名空间中的_1占位符，用于回调函数

// 定义一个名为SimpleParameter的类，继承自rclcpp::Node
class SimpleParameter : public rclcpp::Node
{
public:
    // 构造函数
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_param", 28);                   // 声明一个整数类型的参数，默认值为28
        declare_parameter<std::string>("simple_string_param", "Antonio"); // 声明一个字符串类型的参数，默认值为"Antonio"
        // 添加一个设置参数的回调函数，当参数发生变化时调用
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    // 声明一个设置参数回调处理的共享指针
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 定义设置参数变化的回调函数
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        // 声明结果消息变量
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param : parameters) // 遍历传递进来的参数列表，检查每个参数的名称和类型
        {
            // 如果找到名为"simple_int_param"且类型为整数的参数
            if (param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is " << param.as_int()); // 打印出新的整数值
                result.successful = true;                                                                            // 设置结果对象为成功
            }
            // 如果找到名为"simple_string_param"且类型为字符串的参数
            if (param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is " << param.as_string()); // 打印出新的字符串值
                result.successful = true;                                                                                  // 设置结果对象为成功
            }
        }

        return result; // 返回结果
    }
};

// 定义主函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                        // 初始化rclcpp
    auto node = std::make_shared<SimpleParameter>(); // 创建SimpleParameter对象
    rclcpp::spin(node);                              // 保持节点运行，等待设置参数
    rclcpp::shutdown();                              // 关闭rclcpp
    return 0;                                        // 程序正常退出
}

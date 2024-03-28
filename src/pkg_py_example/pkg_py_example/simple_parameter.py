import rclpy  # 导入rclpy模块，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from rcl_interfaces.msg import SetParametersResult  # 从rcl_interfaces消息中导入SetParametersResult消息类型
from rclpy.parameter import Parameter  # 从rclpy.parameter模块中导入Parameter类

# 定义一个名为SimpleParameter的类，继承自Node类
class SimpleParameter(Node):
    
    def __init__(self):
        super().__init__("simple_parameter")  # 调用父类的构造函数，创建节点，节点名称为simple_parameter
        self.declare_parameter("simple_int_param", 28)  # 声明一个整数类型的参数，默认值为28
        self.declare_parameter("simple_string_param", "Robot")  # 声明一个字符串类型的参数，默认值为"Robot"

        # 添加一个设置参数的回调函数，当参数发生变化时调用
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    # 定义参数变化的回调函数
    def paramChangeCallback(self, params):
        result = SetParametersResult()  # 创建一个SetParametersResult对象

        # 遍历所有传递进来的参数
        for param in params:
            # 如果参数名称为simple_int_param并且类型为整数
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param changed! New value is %d" % param.value)  # 打印新值
                result.successful = True  # 设置结果成功

            # 如果参数名称为simple_string_param并且类型为字符串
            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param changed! New value is %s" % param.value)  # 打印新值
                result.successful = True  # 设置结果成功

        # 返回结果对象
        return result

# 定义主函数
def main():
    rclpy.init()  # 初始化rclpy
    simple_parameter = SimpleParameter()  # 创建SimpleParameter对象
    rclpy.spin(simple_parameter)  # 保持节点运行，等待设置参数
    simple_parameter.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

# 判断是否为主程序
if __name__ == "__main__":
    main()  # 如果是主程序，则运行主函数
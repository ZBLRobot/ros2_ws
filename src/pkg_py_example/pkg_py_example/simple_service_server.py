import rclpy  # 导入rclpy库，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from pkg_interfaces.srv import AddTwoInts  # 从pkg_interfaces服务接口中导入AddTwoInts服务类型

# 定义一个名为SimpleServiceServer的类，继承自Node类
class SimpleServiceServer(Node):

    def __init__(self):
        super().__init__("simple_service_server")  # 调用父类的构造函数，创建节点，节点名称为simple_service_server
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)  # 创建一个服务，服务类型为AddTwoInts，服务名为add_two_ints，回调函数为serviceCallback
        self.get_logger().info("Service add_two_ints Ready")  # 在日志中输出服务add_two_ints已准备好的信息

    # 定义服务回调函数
    def serviceCallback(self, req, res):
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b))  # 在日志中输出接收到的新的请求信息
        res.sum = req.a + req.b  # 计算两个整数的和，并赋值给响应对象的sum字段
        self.get_logger().info("Returning sum: %d" % res.sum)  # 在日志中输出返回的和
        return res  # 返回响应对象

# 定义主函数
def main():
    rclpy.init()  # 初始化rclpy

    simple_service_server = SimpleServiceServer()  # 创建SimpleServiceServer对象
    rclpy.spin(simple_service_server)  # 保持节点运行，等待服务请求
    
    simple_service_server.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

# 判断是否为主程序
if __name__ == '__main__':
    main()  # 如果是主程序，则运行主函数
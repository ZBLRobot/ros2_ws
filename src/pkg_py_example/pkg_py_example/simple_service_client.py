import sys  # 导入sys模块，用于访问与Python解释器相关的变量和函数
import rclpy  # 导入rclpy模块，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from pkg_interfaces.srv import AddTwoInts  # 从bumperbot_msgs服务中导入AddTwoInts服务类型

# 定义一个名为SimpleServiceClient的类，继承自Node类
class SimpleServiceClient(Node):

    def __init__(self, a, b):
        super().__init__("simple_service_client")  # 调用父类的构造函数，创建节点，节点名称为simple_service_client
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")  # 创建一个客户端，用于调用add_two_ints服务

        # 等待服务服务器可用
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")  # 如果服务不可用，则每秒打印一次等待信息
        
        self.req_ = AddTwoInts.Request()  # 创建一个请求对象
        self.req_.a = a  # 设置请求对象的a字段
        self.req_.b = b  # 设置请求对象的b字段
        self.future_ = self.client_.call_async(self.req_)  # 发送异步请求
        self.future_.add_done_callback(self.responseCallback)  # 添加回调函数，用于处理响应

    # 定义响应回调函数
    def responseCallback(self, future):
        self.get_logger().info('Service Response %d' % future.result().sum)  # 打印服务响应的结果

# 定义主函数
def main():
    rclpy.init()  # 初始化rclpy

    # 检查命令行参数数量是否正确
    if len(sys.argv) != 3:
        print("Wrong number of arguments! Usage: simple_service_client A B")
        return -1

    # 创建SimpleServiceClient对象，传入命令行参数作为两个整数
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin(simple_service_client)  # 保持节点运行，等待服务响应

    simple_service_client.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

# 判断是否为主程序
if __name__ == '__main__':
    main()  # 如果是主程序，则运行主函数
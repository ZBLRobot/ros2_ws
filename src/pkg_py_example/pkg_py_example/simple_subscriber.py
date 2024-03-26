import rclpy  # 导入rclpy库，这是Python的ROS2客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from std_msgs.msg import String  # 从std_msgs.msg模块中导入String消息类型，用于发布和订阅字符串消息

# 定义一个SimpleSubscriber类，继承自Node类
class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")  # 调用父类Node的构造函数，创建一个名为"simple_subscriber"的节点
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)  # 创建一个订阅者，订阅名为"chatter"的话题，消息类型为String，回调函数为msgCallback，队列长度为10
        self.sub_  # 这一行似乎是多余的，可能是代码错误

    # 定义一个msgCallback方法，用于处理接收到的消息
    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)  # 使用get_logger方法获取日志记录器，并打印接收到的消息内容

# 定义一个main函数，作为程序的入口点
def main():
    rclpy.init()  # 初始化rclpy库

    simple_publisher = SimpleSubscriber()  # 创建一个SimpleSubscriber对象
    rclpy.spin(simple_publisher)  # 进入事件循环，处理节点上的回调函数

    simple_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy库

# 检查是否为主程序，如果是则运行main函数
if __name__ == '__main__':
    main()
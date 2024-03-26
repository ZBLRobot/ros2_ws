import rclpy                     # 用于与ROS 2核心进行交互的Python客户端库
from rclpy.node import Node      # 节点父类
from std_msgs.msg import String  # 发布标准的String数据  


class SimplePublisher(Node):

    def __init__(self):  # 构造函数，用于对象初始化
        super().__init__("simple_publisher")                        # 定义节点名称
        self.pub_ = self.create_publisher(String, "chatter", 10)    # 创建发布者，指定了消息类型、话题名称和缓冲区队列大小
        self.counter_ = 0                     # 用于计算发布的消息数量
        self.frequency_ = 1.0                 # 定义在主题中发布消息的频率，初始化为1Hz
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)  # 打印一条info级别的log信息
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)  # 创建一个新的计时器对象，该计时器以指定的频率重复执行一个特定函数

    def timerCallback(self):  # 计时器回调函数，目标是每次执行时在chatter话题中发布一条新消息
        msg = String()              # 分配一个字符串变量
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_  # 构造要发布的字符串
        self.pub_.publish(msg)      # 发布消息
        self.counter_ += 1          # 发布消息后计数器加1


def main():          # 主函数
    rclpy.init()     # 使用rclpy库初始化ROS2接口，并使用它的init函数

    simple_publisher = SimplePublisher()  # 创建一个SimplePublisher类的新对象
    rclpy.spin(simple_publisher)          # 使用rclpy库中定义的spin函数，保持这个节点运行，从而保持定时器和发布者持续活动
    
    simple_publisher.destroy_node()  # 当在终端中按Ctrl+C终止这个节点的执行时，还需要确保simple_publisher节点被正确销毁
    rclpy.shutdown()  # 关闭ROS2接口


if __name__ == '__main__':  # 可执行程序的入口函数
    main()  # 调用主函数

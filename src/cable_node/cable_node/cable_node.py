import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
from rclpy.timer import Timer

class CableNode(Node):
    def __init__(self):
        super().__init__('joy_button_publisher')

        self.joy_subscriber = self.create_subscription(Joy, '/joy2', self.joy_callback, 10)

        self.button_publisher = self.create_publisher(Int8MultiArray, '/duty_cr', 10)

        self.timer = self.create_timer(0.100, self.timer_callback)

        self.duty_cr = Int8MultiArray()
        self.duty_cr.data = [0, 0, 0]

    def joy_callback(self, msg: Joy):
        
        # 三角ボタンで排出・バツボタンで巻取り(添字が異なる可能性あり)
        self.duty_cr.data[0] = (msg.buttons[2] - msg.buttons[0]) * 50

        # 中継機の足回りのduty比を計算(スティック右上が正と想定)
        self.duty_cr.data[1] = int((     msg.axes[1] - msg.axes[3])*50)
        self.duty_cr.data[2] = int((-1 * msg.axes[1] - msg.axes[3])*50)

    def timer_callback(self):
        
        self.button_publisher.publish(self.duty_cr)
        self.get_logger().info(f'Published button state: {self.duty_cr.data}')

def main(args=None):

    rclpy.init(args=args)
    node = CableNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

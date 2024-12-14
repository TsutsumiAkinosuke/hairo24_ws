import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_publisher')

        self.declare_parameter('camera1_path', '/dev/video0')
        self.declare_parameter('camera2_path', '/dev/video2')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 15)

        self.camera1_path = self.get_parameter('camera1_path').get_parameter_value().string_value
        self.camera2_path = self.get_parameter('camera2_path').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.camera1 = cv2.VideoCapture(self.camera1_path)
        self.camera2 = cv2.VideoCapture(self.camera2_path)

        if not self.camera1.isOpened():
            self.get_logger().error(f"Could not open camera1: {self.camera1_path}")
            sys.exit()
        
        if not self.camera2.isOpened():
            self.get_logger().error(f"Could not open camera2: {self.camera2_path}")
            sys.exit()

        self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera1.set(cv2.CAP_PROP_FPS, self.fps)
        self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera2.set(cv2.CAP_PROP_FPS, self.fps)

        self.bridge = CvBridge()

        self.image_publisher1 = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.image_publisher2 = self.create_publisher(Image, 'camera2/image_raw', 10)
        self.image_msg1 = Image()
        self.image_msg2 = Image()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):

        ret1, self.frame1 = self.camera1.read()

        if ret1:
            
            gray1 = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)
            resized1 = cv2.resize(gray1, (self.width, self.height))
            self.image_msg1 = self.bridge.cv2_to_imgmsg(resized1, encoding="mono8")
            self.image_publisher1.publish(self.image_msg1)

        ret2, self.frame2 = self.camera2.read()

        if ret2:
            gray2 = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY)
            resized2 = cv2.resize(gray2, (self.width, self.height))
            self.image_msg2 = self.bridge.cv2_to_imgmsg(resized2, encoding="mono8")
            self.image_publisher2.publish(self.image_msg2)
    
    def close(self):
        self.camera1.release()
        self.camera2.release()

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.close()

if __name__ == '__main__':
    main()

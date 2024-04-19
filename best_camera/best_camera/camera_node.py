import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.frame_width = self.declare_parameter('frame_width', 1024).value
        self.frame_height = self.declare_parameter('frame_height', 768).value
        self.freq = self.declare_parameter('frequency', 30.0).value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        self.reliability = self.declare_parameter('reliability', 'reliable').value
        self.history = self.declare_parameter('history', 'keep_last').value
        self.depth = self.declare_parameter('depth', 5).value

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video1')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

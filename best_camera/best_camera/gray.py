import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Gray(Node):
    def __init__(self):
        super().__init__('gray_node')

        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('thrs1', 127)
        self.camera_topic = self.get_parameter('camera_topic').value
        self.thrs1 = self.get_parameter('thrs1').value

        self.get_logger().info("Gray thrs1 : " + str(self.thrs1))

        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(Image, self.camera_topic, self.callback_img, 10)
        self.subscription
        self.publisher = self.create_publisher(Image, '/gray', 10)


    def gray(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.add(gray, self.thrs1)
        return gray

    def callback_img(self, msg):
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray_img = self.gray(cv_img)
        pub_gray = self.cv_bridge.cv2_to_imgmsg(gray_img, 'mono8')
        self.publisher.publish(pub_gray)

def main():
    rclpy.init()
    node = Gray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
import cv2
import numpy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImgCartoon(Node):
    def __init__(self):
        super().__init__('img_cartoon')

        self.declare_parameter('camera_topic', '/camera')

        self.img_subscriber = self.create_subscription(
            Image,
            '/camera', # 이미지 토픽
            self.image_callback,
            10
        )

        self.img_control = self.create_publisher(Image, '/img_cartoon', 10)

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]
        img2 = cv2.resize(img, (w//2, h//2))
        blr = cv2.bilateralFilter(img2, -1, 20, 7)
        edge = 255 - cv2.Canny(img2, 80, 120)
        edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
        img = cv2.bitwise_and(blr, edge)
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)

        pub_img = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_control.publish(pub_img)


def main():
    rclpy.init()

    node = ImgCartoon()

    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 

        
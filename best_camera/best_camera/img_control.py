import cv2
import numpy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImgControl(Node):
    def __init__(self):
        super().__init__('img_control')

        self.declare_parameter('camera_topic', '/camera')

        self.declare_parameter('bright', 3)
        self.bright = self.get_parameter('bright').value
        self.declare_parameter('contrast', 2.0)
        self.contrast = self.get_parameter('contrast').value
        self.declare_parameter('blur', 0)
        self.blur = self.get_parameter('blur').value

        self.img_subscriber = self.create_subscription(
            Image,
            'camera', # 이미지 토픽
            self.image_callback,
            10
        )

        self.img_control = self.create_publisher(Image, '/img_control', 10)

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.contrast != 0: #float
            img = numpy.clip((1+self.contrast)*img - 128*self.contrast, 0, 255).astype(numpy.uint8)
        
        if self.bright != 0: #int
            img = cv2.add(img, (self.bright, self.bright, self.bright, 0))
        
        if self.blur > 0: #int
            img = cv2.blur(img, (self.blur, self.blur))

        pub_img = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_control.publish(pub_img)


def main():
    rclpy.init()

    node = ImgControl()

    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 

        
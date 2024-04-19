import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImgOptical(Node):
    def __init__(self):
        super().__init__('img_optical')

        self.declare_parameter('camera_topic', '/camera')
        
        self.declare_parameter('step', 16)
        self.step = self.get_parameter('step').value
        print(self.step)

        self.img_subscriber = self.create_subscription(
            Image,
            '/camera', # 이미지 토픽
            self.image_callback,
            10
        )

        self.img_control = self.create_publisher(Image, '/img_optical', 10)

        self.cv_bridge = CvBridge()
        
        self.prevgray = None

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.prevgray is None:
            self.prevgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img = self.filter(img)

        pub_img = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_control.publish(pub_img)

    def filter(self, img):
        step = self.step
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(self.prevgray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        self.prevgray = gray

        h, w = gray.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        for (x1, y1), (_x2, _y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)

        return vis
    


def main():
    rclpy.init()

    node = ImgOptical()

    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('thrs1', 127)
        self.camera_topic = self.get_parameter('camera_topic').value
        self.thrs1 = self.get_parameter('thrs1').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.callback_img,
            10
        )
        self.publisher = self.create_publisher(Image, '/aruco', 10)

        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

    def callback_img(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_frame = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    
        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, self.marker_dict)
    
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                cv2.polylines(
                    cv_img, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                cv2.putText(
                    cv_img,
                    f"id: {ids[0]}",
                    tuple(top_right),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv2.LINE_AA,
                )

        pub_aruco = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        self.publisher.publish(pub_aruco)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

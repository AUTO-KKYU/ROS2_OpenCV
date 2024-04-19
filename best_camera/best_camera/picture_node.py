import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from best_camera_msgs.srv import Save, Record
import os 

class PictureNode(Node):
    def __init__(self):
        super().__init__('picture_node')
        self.save_next_image = False
        self.image_file_name = 'undefined.jpg'
        self.video_file_name = 'undefined.avi'
        self.image_count = 0
        self.video_count = 0

        self.recording = False
        self.bridge = CvBridge()
        self.video_writer = None 

        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.save_service = self.create_service(
            Save, 'camera/save_picture', self.handle_service)

        self.record_service = self.create_service(
            Record, 'camera/record', self.handle_record_service)
        

    def image_callback(self, msg):

        if self.save_next_image:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.image_file_name = os.path.join(self.current_dir, f'image_{self.image_count}.jpg')
            self.image_count += 1

            cv2.imwrite(self.image_file_name, cv_image)
            self.save_next_image = False

        if self.recording:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.video_writer is None:
                height, width, channels = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_file_name = os.path.join(self.current_dir, f'video_{self.video_count}.avi')
                self.video_count += 1
                self.video_writer = cv2.VideoWriter(self.video_file_name, fourcc, 20.0, (width, height))

            self.video_writer.write(cv_image)

    def handle_service(self, request, response):
        self.image_file_name = os.path.join(self.current_dir, request.name)
        self.save_next_image = True
        response.result = True
        return response
    
    def handle_record_service(self, request, response):
        if request.start:
            self.recording = True
            self.video_count = 0  # 새로운 동영상을 시작하므로 카운트 초기화
            response.result = True
            response.message = f"Recording started to file: {self.video_file_name}"
        else:
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            response.result = True
            response.message = "Recording stopped"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    picture_node = PictureNode()
    rclpy.spin(picture_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

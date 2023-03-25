# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from sensor_msgs.msg import CameraInfo  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np

# from detection import DroneDetector
# import yaml
# from yaml.loader import SafeLoader

class ImageDetection(Node):


    def __init__(self):

        super().__init__('image_detection_node')

        # self.camera_info_sub_ = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.listener_callback, 10)
        self.image_sub_ = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        # self.detection_info_pub = self.create_publisher(Image,"detection_info",10)

        # with open('camera_param.yaml') as file:
        #     config = yaml.load(file, Loader=SafeLoader)

        # self.detector_ = DroneDetector()
        # self.br_ = CvBridge()

    def image_callback(self, msg):
        # processed_image = self.detector_(msg)
        # self.detection_info_pub.publish(processed_image)
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Convert image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Display grayscale and HSV images
        cv2.imshow("Grayscale Image", gray_image)
        cv2.imshow("HSV Image", hsv_image)
        cv2.waitKey(1)


    # def listener_callback(self, data):


    #     self.get_logger().info('Receiving video frame')

    #     current_frame = self.br_.imgmsg_to_cv2(data)


    #     cv2.imshow("camera", current_frame)

    #     cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_detection = ImageDetection()

    rclpy.spin(image_detection)

    image_detection.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
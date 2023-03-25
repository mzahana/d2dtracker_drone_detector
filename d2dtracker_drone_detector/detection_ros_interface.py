# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from sensor_msgs.msg import CameraInfo  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np

from opencv_test import FaceDetector
from detection import DroneDetector
import yaml
from yaml.loader import SafeLoader


class ImageDetection(Node):

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_detection_node')

        # ROS Independent Variables
        self.timer_period = 0.1  # seconds

        # ROS Dependent Variables



        # Create the subscriber. This subscriber will take information about depth camera
        cam = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.listener_callback, 10)

        # Create the subscriber. This subscriber will listen to Image messages from depth camera
        image = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback, 10)

        
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)


        # Create the subscriber. This subscriber will listen to Image messages
        # on the video_frames topic. The queue size is 10 messages.
        self.subscription_ = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)

        # We will publish a message every 0.1 seconds
        # Create the timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap_ = cv2.VideoCapture(0)

        # Used to convert between ROS and OpenCV images
        self.br_ = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br_.imgmsg_to_cv2(data)

        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap_.read()

        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br_.cv2_to_imgmsg(frame))

            # Display the message on the console
            self.get_logger().info('Publishing video frame')

    # Remove the following two lines since they are not needed
    # The listener_callback and timer_callback methods are instance methods
    # and should be called on the ImageDetection object
    # ImageDetection.listener_callback()
    # ImageDetection.timer_callback()


def main(args=None):

    # img = 
    # caminfo =

    # DroneDetector(img,caminfo)

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_detection = ImageDetection()

    # Spin the node so the callback function is called.
    rclpy.spin(image_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_detection.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
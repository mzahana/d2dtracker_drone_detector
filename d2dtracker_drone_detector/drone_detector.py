import sys

import rclpy
from rclpy.node import Node


class DroneDetector(Node):

    def __init__(self):
        super().__init__('drone_detector')

        with open('/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/camera_param.yaml', 'r') as f:
            data = yaml.load(f, Loader=SafeLoader)

        self.subscription = self.create_subscription(Image,self.img_topic_,self.depthCb,10)
        self.subscription = self.create_subscription(CameraInfo,self.camInfo_topic_,self.camInfoCb,10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(PoseArray, 'drone_detections', 10)
        timer_period = 0.5  # seconds
        self.publisher_ = self.create_publisher(Image, 'detections_image', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from .detection import DroneDetector
import tf2_ros
from tf2_ros import TransformStamped, TransformListener
import tf2_ros.buffer
from geometry_msgs.msg import PoseArray, PointStamped, Pose, TransformStamped
import cv2
import yaml
from yaml.loader import SafeLoader

class DepthCameraNode(Node):

    def __init__(self):
        super().__init__("depth_camera_node")
        self.cv_bridge = CvBridge()

        # @ Read params
        with open('/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/detection_param.yaml', 'r') as f:
            params = yaml.load(f, Loader=SafeLoader)

        self.area_bounds_ =  params["area_bounds"] #[100, 1e4] # in pixels
        self.circ_bounds_ = params["circular_bounds"] # from 0 to 1
        self.conv_bounds_ = params["convexity_bounds"] # from 0 to 1
        self.d_group_max_ = params["d_group_max"] # maximal contour grouping distance in pixels
        self.min_group_size_ = params["min_group_size"] # minimal number of contours for a group to be valid
        self.max_cam_depth_ = params["max_cam_depth"] # Maximum acceptable camera depth values
        self.depth_scale_factor_ = params["depth_scale_factor"] # Scaling factor to make depth values in meters
        self.depth_step_ = params["depth_step"]
        self.debug_ = params["debug"]
        
        # params = data["ros__parameters"]
        # params = {"area_bounds": data["ros__parameters"]["area_bounds"], "circular_bounds": data["ros__parameters"]["circular_bounds"], "convexity_bounds": data["ros__parameters"]["convexity_bounds"], "d_group_max": data["ros__parameters"]["d_group_max"], "min_group_size": data["ros__parameters"]["min_group_size"], "max_cam_depth": data["ros__parameters"]["max_cam_depth"], "depth_scale_factor": data["ros__parameters"]["depth_scale_factor"], "depth_step": data["ros__parameters"]["depth_step"]}

        # Class member 'detector'
        self.detector_= DroneDetector(self.area_bounds_,self.circ_bounds_,self.conv_bounds_,self.d_group_max_,self.min_group_size_,self.max_cam_depth_,self.depth_scale_factor_,self.depth_step_ ,self.debug_)

        self.declare_parameter('debug', True)
        self.debug_ = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('show_debug_images', True)
        self.show_debug_images_ = self.get_parameter('show_debug_images').get_parameter_value().bool_value

        self.declare_parameter('publish_processed_images', True)
        self.pub_processed_images_ =self.get_parameter('publish_processed_images').get_parameter_value().bool_value

        self.declare_parameter('reference_frame', 'map')
        self.reference_frame_ =self.get_parameter('reference_frame').get_parameter_value().string_value


        # Subscribe to image topic
        self.image_sub_ = self.create_subscription(Image,"camera/depth/image_rect_raw",self.imageCallback,10)

        # Subscribe to camera info topic
        self.caminfo_sub_ = self.create_subscription(CameraInfo, 'camera/depth/camera_info', self.caminfoCallback, 10)

        # Publisher of detections positions
        # self.detections_pub_ = rospy.Publisher('drone_detections', PoseArray, queue_size=10)

        # Publisher of image with overlayed detections
        # self.img_pub_ = rospy.Publisher('detections_image', Image, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer,self)

        

    def imageCallback(self, msg):
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")#"32FC1")
        except Exception as e:
            self.get_logger().error("ros_to_cv conversion error {}".format(e))
            return
        
        # @TODO 
        # # @TODO Get latest transform for syncronization confirmation
        # try:
        #     # Get latest transform
        #     t = self.tf_listener_.getLatestCommonTime(self.reference_frame_, msg.header.frame_id)
        #     trans, rot = self.tf_listener_.lookupTransform(self.reference_frame_, msg.header.frame_id, t)
        # except Exception as e:
        #     self.get_logger().error("get transform error {}".format(e))
        #     # rclpy.logerr_throttle(1, e)
        #     return

        try:            
            # Pre-process depth image and extracts contours and their features
            valid_detections, valid_depths, detections_img = self.detector_.preProcessing(cv_image)

            # 3D projections
            positions = self.detector_.depthTo3D(valid_detections, valid_depths)
            if self.debug_:
                self.get_logger().log("3D positions: {}".format(positions))
                # rospy.loginfo_throttle(1, '3D positions: {}'.format(positions)) #find ros2 way to log periodically
                pass

            print("lllllllllllllllllllllll")
            print(positions)
            cv2.imshow("detections_img", detections_img)
            cv2.imshow("positions", positions)
            cv2.waitKey(1)

            # # Transform positions to a reference frame
            # transform = tf2_ros.concatenate_matrices(tf2_ros.translation_matrix(trans), tf2_ros.quaternion_matrix(rot))
            # pose_array = PoseArray()
            # pose_array = self.transformPositions(positions, self.reference_frame_, msg.header.frame_id, msg.header.stamp, transform)

            # if len(pose_array.poses) > 0:
            #     self.detections_pub_.publish(pose_array)

            # if(self.pub_processed_images_):
            #     ros_img = self.bridge_.cv2_to_imgmsg(detections_img)
            #     self.img_pub_.publish(ros_img)
                
        except Exception as e:
            self.get_logger().error("pre processing error {}".format(e))
        # end of @TODO


        # # Convert image to grayscale
        # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # # Convert image to HSV
        # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # # Display grayscale and HSV images
        # cv2.imshow("Grayscale Image", gray_image)
        # cv2.imshow("HSV Image", hsv_image)
        # cv2.imshow("Original Image", cv_image)
        # cv2.waitKey(1)

    def caminfoCallback(self,msg):
        # TODO : fill self.camera_info_ field
        P = np.array(msg.p)
        if len(P) == 12: # Sanity check
            P = P.reshape((3,4))
            self.detector_.camera_info_ = {'fx': P[0][0], 'fy': P[1][1], 'cx': P[0][2], 'cy': P[1][2]}

    # def transformPositions(self, positions, parent_frame, child_frame, tf_time):
    #     """
    #     @brief Converts 3D positions in the camera frame to a parent frame e.g /map
    #     @param positions: List of 3D positions in the child frame (sensor e.g. camera)
    #     @param parent_frame: Frame to transform positions to
    #     @param child_frame: Current frame of positions
    #     @param tf_time: Time at which positions were computed
    #     @return pose_array: PoseArray of all transformed positions
    #     """
    #     pose_array = PoseArray()
    #     pose_array.header.frame_id = parent_frame
    #     pose_array.header.stamp = tf_time
        
    #     for pos in positions:
    #         point = PointStamped()
    #         point.header.frame_id = child_frame
    #         point.header.stamp = tf_time
    #         point.point.x = pos[0]
    #         point.point.y = pos[1]
    #         point.point.z = pos[2]
    #         try:
    #             p = self.tf_listener_.transformPoint(parent_frame,point)
    #         except (tfs.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rclpy.logwarn_throttle(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
    #                                                                                     child_frame,
    #                                                                                     parent_frame))
    #             continue

    #         pose = Pose()
    #         pose.position.x = p.point.x
    #         pose.position.y = p.point.y
    #         pose.position.z = p.point.z
    #         pose.orientation.w = 1.0

    #         pose_array.poses.append(pose)

    #     return pose_array

    # def transformPositions(self, positions, parent_frame, child_frame, tf_time, tr):
    #     """
    #     @brief Converts 3D positions in the camera frame to the parent_frame
    #     @param positions: List of 3D positions in the child frame (sensor e.g. camera)
    #     @param parent_frame: Frame to transform positions to
    #     @param child_frame: Current frame of positions
    #     @param tf_time: Time at which positions were computed
    #     @param tr: Transform 4x4 matrix theat encodes rotation and translation
    #     @return pose_array: PoseArray of all transformed positions
    #     """
    #     pose_array = PoseArray()
    #     pose_array.header.frame_id = parent_frame
    #     pose_array.header.stamp = tf_time
        
    #     for pos in positions:
    #         point = PointStamped()
    #         point.header.frame_id = child_frame
    #         point.header.stamp = tf_time
    #         point.point.x = pos[0]
    #         point.point.y = pos[1]
    #         point.point.z = pos[2]

    #         # Construct homogenous position
    #         pos_h = np.array([pos[0], pos[1], pos[2], 1.0])
    #         # Apply transform
    #         mapped_pos_h = np.dot(tr, pos_h)
    #         # Extract 3D position
    #         mapped_pos = mapped_pos_h[:3]
    #         # try:
    #         #     p = self.tf_listener_.transformPoint(parent_frame,point)
    #         # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         #     rclpy.logwarn_throttle(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
    #         #                                                                             child_frame,
    #         #                                                                             parent_frame))
    #         #     continue

    #         pose = Pose()
    #         pose.position.x = mapped_pos[0]
    #         pose.position.y = mapped_pos[1]
    #         pose.position.z = mapped_pos[2]
    #         pose.orientation.w = 1.0

    #         pose_array.poses.append(pose)

    #     return pose_array


def main(args=None):
    rclpy.init(args=args)
    depth_camera_node = DepthCameraNode()
    rclpy.spin(depth_camera_node)
    depth_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

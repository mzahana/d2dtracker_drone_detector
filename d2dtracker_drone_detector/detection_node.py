#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from .detection import DroneDetector
import tf2_py
from tf2_ros import TransformStamped, TransformListener, LookupTransform
import tf2_ros.buffer
from geometry_msgs.msg import PoseArray, PointStamped, Pose, TransformStamped
import tf_transformations
from tf_transformations import transforms3d as tfs
import tf2_geometry_msgs.tf2_geometry_msgs

class DepthCameraNode(Node):

    def __init__(self):
        # @ Initiate the node
        super().__init__("depth_camera_node")
        # @ Initiate the CV bridge
        self.cv_bridge_ = CvBridge()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('area_bounds',[390,1000]),
                ('circular_bounds',[0.3, 0.99]),
                ('convexity_bounds', [0.7,0.1]),
                ('d_group_max', 50),
                ('min_group_size',4),
                ('max_cam_depth', 20.0),
                ('depth_scale_factor',1.0),
                ('depth_step', 2.0),
                ('debug', True),
                ('show_debug_images', True),
                ('publish_processed_images', True),
                ('reference_frame', 'map'),
            ]
        )

        # Declare ros params
        self.area_bounds_ = self.get_parameter('area_bounds').get_parameter_value().integer_array_value
        self.circ_bounds_ = self.get_parameter('circular_bounds').get_parameter_value().double_array_value
        self.conv_bounds_ = self.get_parameter('convexity_bounds').get_parameter_value().double_array_value
        self.d_group_max_ = self.get_parameter('d_group_max').get_parameter_value().integer_value
        self.min_group_size_ = self.get_parameter('min_group_size').get_parameter_value().integer_value
        self.max_cam_depth_ = self.get_parameter('max_cam_depth').get_parameter_value().double_value
        self.depth_scale_factor_ = self.get_parameter('depth_scale_factor').get_parameter_value().integer_value
        self.depth_step_ = self.get_parameter('depth_step').get_parameter_value().integer_value
        self.debug_ = self.get_parameter('debug').get_parameter_value().bool_value
        self.show_debug_images_ = self.get_parameter('show_debug_images').get_parameter_value().bool_value
        self.pub_processed_images_ =self.get_parameter('publish_processed_images').get_parameter_value().bool_value
        self.reference_frame_ =self.get_parameter('reference_frame').get_parameter_value().string_value

        # Initiate class member 'detector'
        self.detector_= DroneDetector(self.area_bounds_,self.circ_bounds_,self.conv_bounds_,self.d_group_max_,self.min_group_size_,self.max_cam_depth_,self.depth_scale_factor_,self.depth_step_ ,self.debug_)

        # Subscribe to image topic
        self.image_sub_ = self.create_subscription(Image,"interceptor/depth_image",self.imageCallback,10)
        # Subscribe to camera info topic
        self.caminfo_sub_ = self.create_subscription(CameraInfo, 'interceptor/camera_info', self.caminfoCallback, 10)

        # Publish detections positions
        self.detections_pub_ = self.create_publisher(PoseArray,'drone_detections',10)
        # Publish image with overlayed detections
        self.img_pub_ = self.create_publisher( Image,'detections_image',10)

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_,self)

    def imageCallback(self, msg: Image):
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="32FC1")#"16UC1")
        except Exception as e:
            self.get_logger().error("ros_to_cv conversion error {}".format(e))
            return
        
        try:
            # Get latest transform
            # t = self.get_clock().now() #self.tf_buffer_.getLatestCommonTime(self.reference_frame_, msg.header.frame_id)
            # @todo FIX ME
            trans, rot = self.tf_buffer_.lookup_transform(self.reference_frame_, msg.header.frame_id, 0)
        except Exception as e:
            self.get_logger().error("get transform error {}".format(e))
            # rclpy.logerr_throttle(1, e)
            return

        try:            
            # Pre-process depth image and extracts contours and their features
            valid_detections, valid_depths, detections_img = self.detector_.preProcessing(cv_image)
        except Exception as e:
            self.get_logger().error("Error in preProcessing: {}".format(e))
            return

        try:
            # 3D projections
            positions = self.detector_.depthTo3D(valid_detections, valid_depths)
            if self.debug_:
                self.get_logger().info("3D positions: {}".format(positions), throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error("Error in depthTo3D: {}".format(e))
            return

        try:   
            # Transform positions to a reference frame
            transform = tf_transformations.concatenate_matrices(tf_transformations.translation_matrix(trans), tf_transformations.quaternion_matrix(rot))
            tf2_ros.LookupTransform
            pose_array = PoseArray()
            pose_array = self.transformPositions(positions, self.reference_frame_, msg.header.frame_id, msg.header.stamp, transform)
        except Exception as e:
            self.get_logger().error("Error in transforming positions: {}".format(e))
            return

        if len(pose_array.poses) > 0:
            self.detections_pub_.publish(pose_array)

        if(self.pub_processed_images_):
            ros_img = self.cv_bridge_.cv2_to_imgmsg(detections_img)
            self.img_pub_.publish(ros_img)
                

    def caminfoCallback(self,msg: CameraInfo):
        # TODO : fill self.camera_info_ field
        P = np.array(msg.p)
        if len(P) == 12: # Sanity check
            P = P.reshape((3,4))
            self.detector_.camera_info_ = {'fx': P[0][0], 'fy': P[1][1], 'cx': P[0][2], 'cy': P[1][2]}

    def transformPositions(self, positions, parent_frame, child_frame, tf_time):
        """
        @brief Converts 3D positions in the camera frame to a parent frame e.g /map
        @param positions: List of 3D positions in the child frame (sensor e.g. camera)
        @param parent_frame: Frame to transform positions to
        @param child_frame: Current frame of positions
        @param tf_time: Time at which positions were computed
        @return pose_array: PoseArray of all transformed positions
        """
        pose_array = PoseArray()
        pose_array.header.frame_id = parent_frame
        pose_array.header.stamp = tf_time
        
        for pos in positions:
            point = PointStamped()
            point.header.frame_id = child_frame
            point.header.stamp = tf_time
            point.point.x = pos[0]
            point.point.y = pos[1]
            point.point.z = pos[2]
            try:
                # p = tf2_ros.BufferInterface.transform(point,parent_frame)
                p = tf2_geometry_msgs.do_transform_pose(point, parent_frame)
            except Exception as e:#(tf2.LookupException, tf2_ros.BufferInterface.transform.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().error("get transform error {}".format(e))
                # rclpy.logwarn_throttle(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
                #                                                                         child_frame,
                #                                                                         parent_frame))
                continue

            pose = Pose()
            pose.position.x = p.position.x #p.point.x
            pose.position.y = p.position.y #p.point.y
            pose.position.z = p.position.z #p.point.z
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

        return pose_array

    def transformPositions(self, positions, parent_frame, child_frame, tf_time, tr):
        """
        @brief Converts 3D positions in the camera frame to the parent_frame
        @param positions: List of 3D positions in the child frame (sensor e.g. camera)
        @param parent_frame: Frame to transform positions to
        @param child_frame: Current frame of positions
        @param tf_time: Time at which positions were computed
        @param tr: Transform 4x4 matrix theat encodes rotation and translation
        @return pose_array: PoseArray of all transformed positions
        """
        pose_array = PoseArray()
        pose_array.header.frame_id = parent_frame
        pose_array.header.stamp = tf_time
        
        for pos in positions:
            point = PointStamped()
            point.header.frame_id = child_frame
            point.header.stamp = tf_time
            point.point.x = pos[0]
            point.point.y = pos[1]
            point.point.z = pos[2]

            # Construct homogenous position
            pos_h = np.array([pos[0], pos[1], pos[2], 1.0])
            # Apply transform
            mapped_pos_h = np.dot(tr, pos_h)
            # Extract 3D position
            mapped_pos = mapped_pos_h[:3]
            try:
                p = tf2_geometry_msgs.do_transform_pose(parent_frame,point) #self.tf_listener_.transformPoint(parent_frame,point)
            except Exception as e:#(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.get_logger().error("get transform error {}".format(e))
                # rclpy.logwarn_throttle(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
                #                                                                         child_frame,
                #                                                                         parent_frame))
                continue

            pose = Pose()
            pose.position.x = mapped_pos[0]
            pose.position.y = mapped_pos[1]
            pose.position.z = mapped_pos[2]
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

        return pose_array


def main(args=None):
    rclpy.init(args=args)
    depth_camera_node = DepthCameraNode()
    rclpy.spin(depth_camera_node)
    depth_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
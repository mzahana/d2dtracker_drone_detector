#!/usr/bin/env python3

"""
@Description

This node receives 
    vision_msgs/msg/Detection2dArray msg
    Depth image msg sensor_msgs/msg/Image
and converts the 2D Yolo detection to a 3D position as
    geometry_msgs/msg/PoseArray

Author: Mohamed Abdelkader
Contact: mohamedashraf123@gmail.com

"""

import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
# from vision_msgs.msg import Detection2DArray, Detection2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import Pose as TF2Pose

from geometry_msgs.msg import PoseArray, PointStamped, Pose, TransformStamped
from tf2_geometry_msgs import do_transform_point, do_transform_pose

class Yolo2PoseNode(Node):

    def __init__(self):
        # Initiate the node
        super().__init__("yolo2pose_node")

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', True),
                ('publish_processed_images', True),
                ('reference_frame', 'map'),
            ]
        )

        # Get parameters
        self.debug_ = self.get_parameter('debug').get_parameter_value().bool_value
        self.publish_processed_images_ = self.get_parameter('publish_processed_images').get_parameter_value().bool_value
        self.reference_frame_ = self.get_parameter('reference_frame').get_parameter_value().string_value

        self.cv_bridge_ = CvBridge()

        # Camera intrinsics
        self.camera_info_ = None
        self.yolo_detections_msg_ = DetectionArray()
        self.detection_pose_msg_ = PoseArray()

        # Last detection time stamp in seconds
        self.last_detection_t_ = 0.0

        # Subscribers
        self.image_sub_ = self.create_subscription(Image,"interceptor/depth_image",self.depthCallback, qos_profile_sensor_data)
        self.caminfo_sub_ = self.create_subscription(CameraInfo, 'interceptor/camera_info', self.caminfoCallback, 10)
        self.detections_sub_ = self.create_subscription(DetectionArray, 'detections', self.detectionsCallback, 10)

        # Publishers
        self.poses_pub_ = self.create_publisher( PoseArray,'yolo_poses',10)

        # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_,self)

    def depthCallback(self, msg: Image):
        # Check if we have detections at all
        yolo_msg = self.yolo_detections_msg_
        if len(yolo_msg.detections) < 1:
            if(self.debug_):
                self.get_logger().warn("[Yolo2PoseNode::depthCallback] No Yolo detections. Return")
            return
        
        if self.camera_info_ is None:
            if(self.debug_):
                self.get_logger().warn("[Yolo2PoseNode::depthCallback] camera_info is None. Return")
            return
        
        try:
            transform = self.tf_buffer_.lookup_transform(
                self.reference_frame_,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().error(
                f'[Yolo2PoseNode::depthCallback] Could not transform {self.reference_frame_} to {msg.header.frame_id}: {ex}')
            return
        
        current_detection_t = float(yolo_msg.header.stamp.sec) + \
                                float(yolo_msg.header.stamp.nanosec)/1e9
        
        # Check if we have new measurements
        if current_detection_t <= self.last_detection_t_:
            self.get_logger().warn("[Yolo2PoseNode::depthCallback] No new detections. Return")
            return
        self.last_detection_t_ = current_detection_t

        """@TODO
        * Loop over the detections
        * For each detection, find the centroid (or pixel with minimum depth?) of the bounding box in the corresponding depth image
        * Compute the corresponding 3D location using self.camera_info_ and depth (average depth of bbx, min depth of bbx, depth of centroid?)
        * Transform the poses from the camera frame  to the self.reference_frame_
        * Finally, publish the poses 
        """

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge_.imgmsg_to_cv2(msg) #, desired_encoding="32FC1")#"16UC1")
        except Exception as e:
            self.get_logger().error("[Yolo2PoseNode::depthCallback] Image to CvImg conversion error {}".format(e))
            return

        obj = Detection()
        # obj.bbox.center.position.x
        # obj.bbox.center.position.y
        # obj.bbox.size_x
        # obj.bbox.size_y
        
        poses_msg = PoseArray()
        poses_msg.header = yolo_msg.header
        poses_msg.header.frame_id = self.reference_frame_

        for obj in yolo_msg.detections:
            x = int(obj.bbox.center.position.x - obj.bbox.size.x/2)
            y = int(obj.bbox.center.position.y - obj.bbox.size.y/2)
            w = int(obj.bbox.size.x)
            h = int(obj.bbox.size.y)
            roi = cv_image[y:y+h, x:x+w]
            # Minimum depth value in the region of interest rio
            try:
                min_depth = min_value = np.min(roi)
            except Exception as e:
                if self.debug_:
                    self.get_logger().error("[Yolo2PoseNode::depthCallback] Could not compute minimum depth {}".format(e))
                continue

            # bbx center
            pixel = [x,y]

            pose_msg = self.depthToPoseMsg(pixel, min_depth)
            transformed_pose_msg = self.transformPose(pose_msg, self.reference_frame_, msg.header.frame_id, msg.header.stamp, transform)
            poses_msg.poses.append(transformed_pose_msg)

        if(len(poses_msg.poses)>0):
            self.poses_pub_.publish(poses_msg)
        else:
            if self.debug_:
                self.get_logger().warn("[Yolo2PoseNode::depthCallback] No poses to publish")

        

    def caminfoCallback(self,msg: CameraInfo):
        # TODO : fill self.camera_info_ field
        P = np.array(msg.p)
        K = np.array(msg.k)
        # if len(P) == 12: # Sanity check
        #     P = P.reshape((3,4))
        #     self.camera_info_ = {'fx': P[0][0], 'fy': P[1][1], 'cx': P[0][2], 'cy': P[1][2]}

        if len(K) == 9: # Sanity check
            K = K.reshape((3,3))
            self.camera_info_ = {'fx': K[0][0], 'fy': K[1][1], 'cx': K[0][2], 'cy': K[1][2]}

    def detectionsCallback(self, msg: DetectionArray):
        self.yolo_detections_msg_ = msg

    def depthToPoseMsg(self, pixel, depth):
        """
        @brief Computes 3D projections of detections in the camera frame (+X-right, +y-down, +Z-outward)
        @param pixel : xy coordinates in 2D camera frame
        @param depth : Value of pixel
        @return position : 3D projections in camera frame
        """
        pose_msg = Pose()
        if self.camera_info_ is None:
            print("[Yolo2PoseNode::depthToPoseMsg] Camera intrinsic parameters are not available. Skipping 3D projections.")
            return pose_msg

        fx = self.camera_info_['fx']
        fy = self.camera_info_['fy']
        cx = self.camera_info_['cx']
        cy = self.camera_info_['cy']
        u = pixel[0] # horizontal image coordinate
        v = pixel[1] # vertical image coordinate
        d = depth # depth

        x = d*(u-cx)/fx
        y = d*(v-cy)/fy

        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = float(d)
        pose_msg.orientation.w = 1.0
        

        return pose_msg
    
    def transformPose(self, pose: Pose, parent_frame: str, child_frame: str, tf_time, tr: TransformStamped) -> Pose:
        """
        @brief Converts 3D positions in the camera frame to the parent_frame
        @param pose:  3D position in the child frame (sensor e.g. camera)
        @param parent_frame: Frame to transform positions to
        @param child_frame: Current frame of positions
        @param tf_time: Time at which positions were computed
        @param tr: Transform 4x4 matrix theat encodes rotation and translation
        @return transformed_pose: Pose of transformed position
        """     
        tf2_pose_msg = TF2Pose()
        tf2_pose_msg.position.x = pose.position.x
        tf2_pose_msg.position.y = pose.position.y
        tf2_pose_msg.position.z = pose.position.z
        tf2_pose_msg.orientation.w = 1.0        
        
        try:
            transformed_pose = do_transform_pose(tf2_pose_msg, tr)
        except Exception as e:
            self.get_logger().error("[transformPose] Error in transforming point {}".format(e))
            return None

            

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)
    yolo2pose_node = Yolo2PoseNode()
    yolo2pose_node.get_logger().info("Yolo to Pose conversion node has started")
    rclpy.spin(yolo2pose_node)
    yolo2pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()        
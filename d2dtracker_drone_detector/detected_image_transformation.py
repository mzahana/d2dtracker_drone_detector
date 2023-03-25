#!/usr/bin/env python3

import numpy as np
import cv2
import math
import time
from detection import DroneDetector

# class DetectionTransformation:
#     def __init__(self,img,camInfo,params):
#         self.img_topic_ = img
#         self.camInfo_topic_ = camInfo
    

#         """
#         Contour constraints
#         """



#         self.area_bounds_ = params['detection_parameters']['area_bounds']#[100, 1e4] # in pixels
#         self.circ_bounds_ = params['detection_parameters']['circ_bounds']# from 0 to 1
#         self.conv_bounds_ = params['detection_parameters']['conv_bounds']# from 0 to 1
#         self.d_group_max_ = params['detection_parameters']['d_group_max']# maximal contour grouping distance in pixels
#         self.min_group_size_ = params['detection_parameters']['min_group_size']# minimal number of contours for a group to be valid

#         self.max_cam_depth_ = params['cam__parameters']['max_cam_depth']# Maximum acceptable camera depth values
#         self.depth_scale_factor_ = params['cam__parameters']['depth_scale_factor']# Scaling factor to make depth values in meters
#         self.depth_step_ = params['cam__parameters']['depth_step']

#     def depthCb(self, msg):
#         try:
#             # cv_image has pixel values in meters using the '32FC1' encoding
#             # You can check by printing cv_image.min() and cv_image.max()
#             cv_image = self.bridge_.imgmsg_to_cv2(msg, "32FC1")
#         except Exception as e:
#             print("e")
#             return
        
#         try:
#             # Get latest transform
#             t = self.tf_listener_.getLatestCommonTime(self.reference_frame_, msg.header.frame_id)
#             trans, rot = self.tf_listener_.lookupTransform(self.reference_frame_, msg.header.frame_id, t)
#         except Exception as e:
#             print(1, e)
#             return
        
#         try:            
#             # Pre-process depth image and extracts contours and their features
#             valid_detections, valid_depths, detections_img = self.preProcessing(cv_image)

#             # 3D projections
#             positions = self.depthTo3D(valid_detections, valid_depths)

#             # Transform positions to a reference frame
#             transform = tfs.concatenate_matrices(tfs.translation_matrix(trans), tfs.quaternion_matrix(rot))
#             pose_array = PoseArray()
#             pose_array = self.transformPositions(positions, self.reference_frame_, msg.header.frame_id, msg.header.stamp, transform)

#             if len(pose_array.poses) > 0:
#                 self.detections_pub_.publish(pose_array)

#             # if(self.pub_processed_images_):
#             #     ros_img = self.bridge_.cv2_to_imgmsg(detections_img)
#             #     self.img_pub_.publish(ros_img)

#     def camInfoCb(self, msg):
#         # TODO : fill self.camera_info_ field
#         P = np.array(msg.P)
#         if len(P) == 12: # Sanity check
#             P = P.reshape((3,4))
#             self.camera_info_ = {'fx': P[0][0], 'fy': P[1][1], 'cx': P[0][2], 'cy': P[1][2]}

#     def transformPositions(self, positions, parent_frame, child_frame, tf_time):
#         """
#         @brief Converts 3D positions in the camera frame to a parent frame e.g /map
#         @param positions: List of 3D positions in the child frame (sensor e.g. camera)
#         @param parent_frame: Frame to transform positions to
#         @param child_frame: Current frame of positions
#         @param tf_time: Time at which positions were computed
#         @return pose_array: PoseArray of all transformed positions
#         """
#         pose_array = PoseArray()
#         pose_array.header.frame_id = parent_frame
#         pose_array.header.stamp = tf_time
        
#         for pos in positions:
#             point = PointStamped()
#             point.header.frame_id = child_frame
#             point.header.stamp = tf_time
#             point.point.x = pos[0]
#             point.point.y = pos[1]
#             point.point.z = pos[2]
#             # try:
#             #     p = self.tf_listener_.transformPoint(parent_frame,point)
#             # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             #     print(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
#             #                                                                             child_frame,
#             #                                                                             parent_frame))
#             #     continue

#             pose = Pose()
#             pose.position.x = p.point.x
#             pose.position.y = p.point.y
#             pose.position.z = p.point.z
#             pose.orientation.w = 1.0

#             pose_array.poses.append(pose)

#         return pose_array

#     def transformPositions(self, positions, parent_frame, child_frame, tf_time, tr):
#         """
#         @brief Converts 3D positions in the camera frame to the parent_frame
#         @param positions: List of 3D positions in the child frame (sensor e.g. camera)
#         @param parent_frame: Frame to transform positions to
#         @param child_frame: Current frame of positions
#         @param tf_time: Time at which positions were computed
#         @param tr: Transform 4x4 matrix theat encodes rotation and translation
#         @return pose_array: PoseArray of all transformed positions
#         """
#         pose_array = PoseArray()
#         pose_array.header.frame_id = parent_frame
#         pose_array.header.stamp = tf_time
        
#         for pos in positions:
#             point = PointStamped()
#             point.header.frame_id = child_frame
#             point.header.stamp = tf_time
#             point.point.x = pos[0]
#             point.point.y = pos[1]
#             point.point.z = pos[2]

#             # Construct homogenous position
#             pos_h = np.array([pos[0], pos[1], pos[2], 1.0])
#             # Apply transform
#             mapped_pos_h = np.dot(tr, pos_h)
#             # Extract 3D position
#             mapped_pos = mapped_pos_h[:3]
#             # try:
#             #     p = self.tf_listener_.transformPoint(parent_frame,point)
#             # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             #     rclpy.logwarn_throttle(1, "[{0}] TF Error tranforming point from {1} to {2}".format(rclpy.get_name(),
#             #                                                                             child_frame,
#             #                                                                             parent_frame))
#             #     continue

#             pose = Pose()
#             pose.position.x = mapped_pos[0]
#             pose.position.y = mapped_pos[1]
#             pose.position.z = mapped_pos[2]
#             pose.orientation.w = 1.0

#             pose_array.poses.append(pose)

#         return pose_array

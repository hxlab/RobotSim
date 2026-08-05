#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Pose

import numpy as np

import cv2
from cv_bridge import CvBridge

import threading

# we want to read from the camera topics (rgb and depth), store the latest image, perform unseen object segmentation using uois, and then generate grasps using the contact graspnet model
# we will use the rclpy library to subscripe to topics, uois for segmentation, and contact graspnet for grasp generation

class GraspGenNode(Node):
    def __init__(self):
        Node.__init__(self, 'grasp_generation')

        if self.is_gazebo == 'true':
            rgb_topic = '/depth_camera/image'
            depth_topic = '/depth_camera/points'
        else:
            rgb_topic = '/camera/camera/color/image_raw'
            depth_topic = '/camera/depth/color/points'

        # storage for the most recent RGB and depth images
        self.latest_rgb_image = None
        self.latest_depth_image = None

        # initialize the CvBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
            
        # subscribe to camera topic
        self.rgb_subscription = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10
        )   
        self.depth_subscription = self.create_subscription(
            PointCloud2,
            depth_topic,
            self.depth_callback,
            10
        )

        # publish the segmentation mask and top grasp pose
        self.segmentation_mask_publisher = self.create_publisher(Image, '/segmentation_mask', 10)
        self.top_grasp_pose_publisher = self.create_publisher(Pose, '/top_grasp_pose', 10)

    def rgb_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self.latest_rgb_image = cv_img_rgb
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def depth_callback(self, msg):
        try:
            # convert the PointCloud2 message to a numpy array
            points = np.frombuffer(msg.data, dtype=np.float32)
            points = points.reshape(-1, 4)
            self.latest_depth_image = points

        except Exception as e:
            self.get_logger().error(f"Failed to convert point cloud: {e}")

    def process_image(self):
        if self.latest_rgb_image is not None and self.latest_depth_image is not None:
            rgb_img_buffer = self.latest_rgb_image.copy()
            depth_img_buffer = self.latest_depth_image.copy()

            # perform unseen object segmentation using uois
            segmentation_mask = self.perform_segmentation(self.latest_rgb_image)
            # publish segmentation mask
            mask_msg = self.bridge.cv2_to_imgmsg(segmentation_mask, encoding='mono8')
            self.segmentation_mask_publisher.publish(mask_msg)

            # generate grasps using contact graspnet
            top_grasp_pose = self.generate_grasps(self.latest_rgb_image, self.latest_depth_image, segmentation_mask)
            # publish top grasp pose
            self.top_grasp_pose_publisher.publish(top_grasp_pose)

    def perform_segmentation(self, rgb_image):
        # Placeholder for unseen object segmentation using uois
        # Replace this with actual segmentation code
        segmentation_mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        return segmentation_mask

    def generate_grasps(self, rgb_image, depth_image, segmentation_mask):
        # Placeholder for grasp generation using contact graspnet
        # Replace this with actual grasp generation code
        top_grasp_pose = Pose()
        return top_grasp_pose

def ros_spin_worker(node):
    """Background thread runner to process incoming ROS 2 camera events."""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = GraspGenNode()
    
    # Fire up a background thread to look for ROS topics without blocking PyQt
    ros_thread = threading.Thread(target=ros_spin_worker, args=(node,), daemon=True)
    ros_thread.start()
    
    # Cleanup smoothly on exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
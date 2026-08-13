#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber

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
        self.rgb_subscription = Subscriber(
            self
            Image,
            rgb_topic,
        )   
        self.depth_subscription = Subscriber(
            self,
            PointCloud2,
            depth_topic,
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_subscription, self.depth_subscription], 
            queue_size=10, 
            slop=0.1
        )

        # 3. Register the single callback function
        self.ts.registerCallback(self.process_data_callback)

        # publish the segmentation mask and top grasp pose
        self.segmentation_mask_publisher = self.create_publisher(Image, '/segmentation_mask', 10)


    def process_data_callback(self, rgb_msg, depth_msg):
        try:
            # convert the imgmsg to opencv format
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self.latest_rgb_image = cv_img_rgb

            # convert the PointCloud2 message to a numpy array
            points = np.frombuffer(msg.data, dtype=np.float32)
            points = points.reshape(-1, 4)
            self.latest_depth_image = points

            # segmentation
            segmentation_mask = self.perform_segmentation(self.latest_rgb_image)
            self.segmentation_mask_publisher.publish(mask_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

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
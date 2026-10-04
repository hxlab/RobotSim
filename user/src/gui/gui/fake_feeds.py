#!/usr/bin/env python3
"""Standalone test publisher for the GUI's three image views.

Publishes distinct moving test patterns on the camera, /segmentation, and
/grasp_overlay topics at 5 Hz, and logs anything received on /reset_pose.
Lets the GUI toggles be verified with no sim, camera, or CGN node running:

    ros2 run gui fake_feeds
    ros2 run gui gui_app
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge


class FakeFeeds(Node):
    def __init__(self):
        super().__init__('fake_feeds')
        self.declare_parameter('camera_topic', '/depth_camera/image')
        camera_topic = self.get_parameter('camera_topic').value

        self.bridge = CvBridge()
        self.camera_pub = self.create_publisher(Image, camera_topic, 10)
        self.seg_pub = self.create_publisher(Image, '/segmentation', 10)
        self.grasp_pub = self.create_publisher(Image, '/grasp_overlay', 10)
        self.reset_sub = self.create_subscription(Empty, '/reset_pose', self.on_reset, 10)
        self.tick = 0
        self.timer = self.create_timer(0.2, self.publish_frames)
        self.get_logger().info(f'Publishing test frames on {camera_topic}, /segmentation, /grasp_overlay')

    def on_reset(self, _msg):
        self.get_logger().info('reset_pose received')

    def frame(self, kind):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        x = (self.tick * 8) % 640
        if kind == 'camera':
            img[:, :, :] = 40
            img[:, x:x + 40, 1] = 255          # moving green bar
        elif kind == 'segmentation':
            img[:240, :320] = (255, 0, 0)       # colored quadrants
            img[:240, 320:] = (0, 255, 0)
            img[240:, :320] = (0, 0, 255)
            img[240:, x:x + 40] = (255, 255, 0)
        else:
            img[:, :, 2] = 90
            img[200:280, x:x + 40, :] = 255     # moving white block on red
        return img

    def publish_frames(self):
        now = self.get_clock().now().to_msg()
        for kind, pub in (('camera', self.camera_pub),
                          ('segmentation', self.seg_pub),
                          ('grasp', self.grasp_pub)):
            msg = self.bridge.cv2_to_imgmsg(self.frame(kind), encoding='bgr8')
            msg.header.stamp = now
            pub.publish(msg)
        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeFeeds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

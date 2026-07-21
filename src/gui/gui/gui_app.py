#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap 
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget, QPushButton
from PyQt5.QtCore import QTimer

from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt
import threading

# we want to read from the camera topic /camera/camera/color/image_raw and display the image in a PyQt5 window
# we will use the rclpy library to subscribe to the topic and the PyQt5 library to create the GUI

class GUINode(Node, QMainWindow):
    image_signal = pyqtSignal(QImage)

    def __init__(self):
        Node.__init__(self, 'gui_app')
        QMainWindow.__init__(self)
        
        self.setWindowTitle('ROS2 GUI Application')
        self.resize(1357, 549)
        self.setMinimumSize(1357, 549)

        # create the label
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.setCentralWidget(self.image_label)

        self.get_logger().info('GUI Node has been started.')
        self.bridge = CvBridge()

        # GUI update function
        self.image_signal.connect(self.update_gui_image)

        # subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )   

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            
            height, width, channels = cv_img_rgb.shape
            bytes_per_line = channels * width
            
            # .copy() forces PyQt to own the memory block so it doesn't get garbage-collected
            q_image = QImage(
                cv_img_rgb.data, 
                width, 
                height, 
                bytes_per_line, 
                QImage.Format.Format_RGB888
            ).copy()

            # Safely emit the image to the main GUI thread
            self.image_signal.emit(q_image)
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    @pyqtSlot(QImage)
    def update_gui_image(self, q_image):
        """This function executes strictly in the main thread when a signal arrives."""
        pixmap = QPixmap.fromImage(q_image)
        # scaled to maintain aspect ratio within your large window dimensions
        scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled_pixmap)

def ros_spin_worker(node):
    """Background thread runner to process incoming ROS 2 camera events."""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    gui = GUINode()
    gui.show()
    
    # Fire up a background thread to look for ROS topics without blocking PyQt
    ros_thread = threading.Thread(target=ros_spin_worker, args=(gui,), daemon=True)
    ros_thread.start()
    
    exit_code = app.exec_()
    
    # Cleanup smoothly on exit
    gui.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
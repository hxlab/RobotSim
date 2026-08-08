#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap 
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QWidget,
    QPushButton,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QSlider,
    QCheckBox,
    QSizePolicy,
)
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
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                margin-top: 12px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 4px;
            }

            QPushButton {
                min-height: 30px;
            }

            QSlider {
                min-height: 25px;
            }
        """)

        self.configure_layout()

        self.get_logger().info('GUI Node has been started.')
        self.bridge = CvBridge()

        # GUI update function
        self.image_signal.connect(self.update_gui_image)
        self.grasp_slider.valueChanged.connect(self.update_grasp_slider)
        self.traj_slider.valueChanged.connect(self.update_traj_slider)
        self.adaptive_checkbox.toggled.connect(self.adaptive_autonomy_changed)

        self.declare_parameter('is_gazebo', 'true')
        self.is_gazebo = self.get_parameter('is_gazebo').get_parameter_value().string_value
        self.get_logger().info(f'Received argument: {self.is_gazebo}')

        if self.is_gazebo == 'true':
            camera_topic = '/depth_camera/image'
        else:
            camera_topic = '/camera/camera/color/image_raw'
            
        # subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
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

    def update_grasp_slider(self, value):
        self.grasp_value.setText(f"{value/100:.2f}")

    def update_traj_slider(self, value):
        self.traj_value.setText(f"{value/100:.2f}")

    def adaptive_autonomy_changed(self, checked):
        self.grasp_slider.setEnabled(not checked)
        self.traj_slider.setEnabled(not checked)

    def configure_layout(self):
        """Configure the layout of the GUI"""
        #################################################
        # Main Layout
        #################################################

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)

        #################################################
        # Left Control Panel
        #################################################

        control_panel = QWidget()
        control_panel.setFixedWidth(300)

        panel_layout = QVBoxLayout(control_panel)
        panel_layout.setSpacing(15)

        #################################################
        # Controller
        #################################################

        controller_group = QGroupBox("Controller")
        controller_layout = QVBoxLayout()

        self.reset_pose_button = QPushButton("Reset Pose")
        controller_layout.addWidget(self.reset_pose_button)

        controller_group.setLayout(controller_layout)
        panel_layout.addWidget(controller_group)

        #################################################
        # Autonomy
        #################################################

        autonomy_group = QGroupBox("Autonomy")
        autonomy_layout = QVBoxLayout()

        #
        # Grasp Assist
        #

        autonomy_layout.addWidget(QLabel("Grasp Assist"))

        grasp_row = QHBoxLayout()

        self.grasp_slider = QSlider(Qt.Horizontal)
        self.grasp_slider.setRange(0, 100)
        self.grasp_slider.setValue(50)

        self.grasp_value = QLabel("0.50")
        self.grasp_value.setFixedWidth(40)

        grasp_row.addWidget(self.grasp_slider)
        grasp_row.addWidget(self.grasp_value)

        autonomy_layout.addLayout(grasp_row)

        #
        # Trajectory Guidance
        #

        autonomy_layout.addWidget(QLabel("Trajectory Guidance"))

        traj_row = QHBoxLayout()

        self.traj_slider = QSlider(Qt.Horizontal)
        self.traj_slider.setRange(0, 100)
        self.traj_slider.setValue(50)

        self.traj_value = QLabel("0.50")
        self.traj_value.setFixedWidth(40)

        traj_row.addWidget(self.traj_slider)
        traj_row.addWidget(self.traj_value)

        autonomy_layout.addLayout(traj_row)

        #
        # Adaptive Autonomy
        #

        self.adaptive_checkbox = QCheckBox("Adaptive Autonomy")
        autonomy_layout.addWidget(self.adaptive_checkbox)

        autonomy_group.setLayout(autonomy_layout)
        panel_layout.addWidget(autonomy_group)

        #################################################
        # Grasp Generation
        #################################################

        grasp_group = QGroupBox("Grasp Generation")
        grasp_layout = QVBoxLayout()

        self.segmentation_checkbox = QCheckBox("Show Object Segmentation")
        self.grasp_checkbox = QCheckBox("Show Grasp Candidate")

        grasp_layout.addWidget(self.segmentation_checkbox)
        grasp_layout.addWidget(self.grasp_checkbox)

        grasp_group.setLayout(grasp_layout)
        panel_layout.addWidget(grasp_group)

        panel_layout.addStretch()

        #################################################
        # Camera View
        #################################################

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        #################################################
        # Add widgets to layout
        #################################################

        main_layout.addWidget(control_panel)
        main_layout.addWidget(self.image_label, 1)

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
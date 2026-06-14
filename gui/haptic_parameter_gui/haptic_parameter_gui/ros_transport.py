from .ros_payload import PARAMETER_UPDATE_TOPIC, serialize_ros_message_payload


try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError as error:
    rclpy = None
    Node = object
    String = None
    ROS_IMPORT_ERROR = error
else:
    ROS_IMPORT_ERROR = None


class ParameterPublisher(Node):
    def __init__(self):
        super().__init__("haptic_parameter_gui")
        self.publisher = self.create_publisher(String, PARAMETER_UPDATE_TOPIC, 10)

    def publish(self, parameters):
        msg = String()
        msg.data = serialize_ros_message_payload(parameters)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published parameters to {PARAMETER_UPDATE_TOPIC}")


def create_ros_transport():
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 Python packages are not available. "
            "Source your ROS 2 environment before using the ROS transport.\n"
            f"Import error: {ROS_IMPORT_ERROR}"
        )

    rclpy.init(args=None)
    return ParameterPublisher()


def shutdown_ros_transport(node):
    node.destroy_node()
    rclpy.shutdown()

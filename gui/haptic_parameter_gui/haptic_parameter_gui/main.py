import argparse
import sys

from PySide6.QtWidgets import QApplication

from .app import ParameterGui
from .console import ConsoleTransport
from .ros_transport import create_ros_transport, shutdown_ros_transport


def parse_args():
    parser = argparse.ArgumentParser(description="Haptic controller parameter GUI")
    parser.add_argument(
        "--transport",
        choices=("console", "ros2"),
        default="console",
        help="Where Apply sends the full parameter object.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    ros_node = None

    if args.transport == "ros2":
        try:
            ros_node = create_ros_transport()
        except RuntimeError as error:
            raise SystemExit(str(error)) from error
        transport = ros_node
    else:
        transport = ConsoleTransport()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ParameterGui(on_apply=transport.publish)
    window.show()

    exit_code = app.exec()
    if ros_node is not None:
        shutdown_ros_transport(ros_node)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

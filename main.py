import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import rclpy
from ros_integration import RosIntegration
from qt_main import MainWindow

def main(args=None):
    rclpy.init(args=args)
    # Create the ROS integration node with multiple robot namespaces.
    ros_interface = RosIntegration(namespace_list=["robot_1", "robot_2"])
    app = QApplication(sys.argv)
    
    # Set up a QTimer to periodically spin the ROS node.
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_interface, timeout_sec=0))
    timer.start(50)
    
    # Create and show the main window.
    window = MainWindow(ros_interface)
    window.show()
    
    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()

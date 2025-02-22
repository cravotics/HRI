import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import rclpy
from ros_integration import RosIntegration
from qt_features import RobotMonitorUI  # Import the main UI

# --- Main Window Class ---
class MainWindow(QMainWindow):
    """
    The main window of the Multiviz application, embedding the RobotMonitorUI.
    """
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.robot_monitor_ui = RobotMonitorUI(ros_interface)
        self.setCentralWidget(self.robot_monitor_ui)
        self.setWindowTitle("Multiviz - Robot Monitor and Control Interface")

    def closeEvent(self, event):
        """
        Handle the window close event to properly shut down ROS 2.
        """
        print("Closing application... Shutting down ROS 2.")
        self.ros_interface.destroy_node()
        rclpy.shutdown()
        event.accept()

# --- Main Function ---
def main(args=None):
    """
    Initializes ROS 2, sets up the PyQt5 application, and starts the main event loop.
    """
    rclpy.init(args=args)

    # Create the ROS integration node with multiple robot namespaces.
    ros_interface = RosIntegration(namespace_list=["robot_1", "robot_2"])

    # Initialize the Qt application.
    app = QApplication(sys.argv)

    # Set up a QTimer to periodically spin the ROS node.
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_interface, timeout_sec=0))
    timer.start(50)

    # Create and display the main application window.
    window = MainWindow(ros_interface)
    window.show()

    # Execute the application event loop.
    ret = app.exec_()

    # Ensure ROS shutdown if app exits unexpectedly.
    rclpy.shutdown()
    sys.exit(ret)

# --- Entry Point ---
if __name__ == '__main__':
    main()

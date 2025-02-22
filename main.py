import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import rclpy
from ros_integration import RosIntegration
from qt_feature import RobotMonitorUI 

class MainWindow(QMainWindow):
    """
    The main window of the Multiviz application, embedding the RobotMonitorUI.
    """
    def __init__(self, ros_interface):
        super().__init__()
        self.robot_monitor_ui = RobotMonitorUI(ros_interface)
        self.setCentralWidget(self.robot_monitor_ui)
        self.setWindowTitle("Multiviz - Robot Monitor and Control Interface")

def main(args=None):
    """
    Initializes ROS 2, sets up the PyQt5 application, and starts the main event loop.
    """
    rclpy.init(args=args)

    ros_interface = RosIntegration()

    app = QApplication(sys.argv)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_interface, timeout_sec=0))
    timer.start(50)

    window = MainWindow(ros_interface)
    window.show()

    ret = app.exec_()

    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
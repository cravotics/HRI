from PyQt5.QtWidgets import QMainWindow
from qt_feature import RobotMonitorUI

class MainWindow(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.robot_monitor_ui = RobotMonitorUI(ros_interface)
        self.setCentralWidget(self.robot_monitor_ui)
        self.setWindowTitle("Multiviz - Robot Monitor and Control Interface")

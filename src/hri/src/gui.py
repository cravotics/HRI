import sys
import rclpy
from rclpy.node import Node
from threading import Thread
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton, QGridLayout, QInputDialog)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QFont, QImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class RobotMonitorUI(QWidget):
    def __init__(self):
        super().__init__()

        # ROS 2 initialization
        rclpy.init(args=None)
        self.node = rclpy.create_node('robot_monitor_ui')
        self.bridge = CvBridge()
        self.namespace = 'waffle1'  # Default robot namespace

        # Initialize the UI layout
        self.initUI()

        # Initialize ROS topics for the default robot
        self.initialize_ros_topics()

        # Start ROS 2 spinning in a separate thread
        self.ros_thread = Thread(target=self.spin_ros)
        self.ros_thread.start()

    def initUI(self):
        # Main layout
        main_layout = QVBoxLayout(self)

        # Create header with robot selection dropdown
        header_layout = QHBoxLayout()

        title_label = QLabel("Robot Monitor")
        title_label.setFont(QFont("Arial", 20, QFont.Bold))
        header_layout.addWidget(title_label)

        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(["waffle1", "waffle2", "waffle3"])
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        header_layout.addWidget(self.robot_dropdown)

        main_layout.addLayout(header_layout)

        # Display sections for topics
        self.odom_value = QLineEdit(self)
        self.odom_value.setReadOnly(True)
        self.odom_value.setPlaceholderText("Odometry Data")
        main_layout.addWidget(QLabel("Odometry:"))
        main_layout.addWidget(self.odom_value)

        self.laser_value = QLineEdit(self)
        self.laser_value.setReadOnly(True)
        self.laser_value.setPlaceholderText("Laser Scan Data")
        main_layout.addWidget(QLabel("Laser Scan:"))
        main_layout.addWidget(self.laser_value)

        self.image_label = QLabel(self)
        self.image_label.setPixmap(QPixmap(320, 240))
        main_layout.addWidget(QLabel("Camera View:"))
        main_layout.addWidget(self.image_label)

        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        self.cmd_vel_value.setPlaceholderText("Command Velocity Data")
        main_layout.addWidget(QLabel("Command Velocity:"))
        main_layout.addWidget(self.cmd_vel_value)

        # Teleoperation controls
        teleop_layout = QGridLayout()

        self.btn_up = QPushButton("↑", self)
        self.btn_up.clicked.connect(self.move_forward)
        teleop_layout.addWidget(self.btn_up, 0, 1)

        self.btn_left = QPushButton("←", self)
        self.btn_left.clicked.connect(self.turn_left)
        teleop_layout.addWidget(self.btn_left, 1, 0)

        self.btn_stop = QPushButton("■", self)
        self.btn_stop.clicked.connect(self.stop_robot)
        teleop_layout.addWidget(self.btn_stop, 1, 1)

        self.btn_right = QPushButton("→", self)
        self.btn_right.clicked.connect(self.turn_right)
        teleop_layout.addWidget(self.btn_right, 1, 2)

        self.btn_down = QPushButton("↓", self)
        self.btn_down.clicked.connect(self.move_backward)
        teleop_layout.addWidget(self.btn_down, 2, 1)

        main_layout.addLayout(teleop_layout)

        self.setLayout(main_layout)
        self.setWindowTitle("Robot Monitor and Teleop")
        self.setGeometry(100, 100, 600, 400)

    def on_robot_selected(self, robot_name):
        self.namespace = robot_name
        self.initialize_ros_topics()

    def initialize_ros_topics(self):
        try:
            self.node.destroy_subscription(self.odom_sub)
            self.node.destroy_subscription(self.scan_sub)
            self.node.destroy_subscription(self.image_sub)
            self.node.destroy_publisher(self.cmd_vel_pub)
        except AttributeError:
            pass

        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.namespace}/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.node.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.scan_callback,
            10
        )

        self.image_sub = self.node.create_subscription(
            Image,
            f'/{self.namespace}/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.odom_value.setText(f"x: {position.x:.2f}, y: {position.y:.2f}")

    def scan_callback(self, msg):
        self.laser_value.setText(f"{msg.ranges[0]:.2f}, {msg.ranges[1]:.2f}, {msg.ranges[2]:.2f}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            q_image = QImage(cv_image.data, width, height, QImage.Format_RGB888).rgbSwapped()
            self.image_label.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            print(f"Failed to process image: {e}")

    def move_forward(self):
        self.publish_cmd_vel(0.2, 0.0)

    def move_backward(self):
        self.publish_cmd_vel(-0.2, 0.0)

    def turn_left(self):
        self.publish_cmd_vel(0.0, 0.5)

    def turn_right(self):
        self.publish_cmd_vel(0.0, -0.5)

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText(f"Linear: {linear}, Angular: {angular}")

    def spin_ros(self):
        rclpy.spin(self.node)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = RobotMonitorUI()
    ex.show()
    sys.exit(app.exec_())

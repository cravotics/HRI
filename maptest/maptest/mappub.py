import sys
import os
import rclpy
from rclpy.node import Node
from threading import Thread
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton, QGridLayout, QInputDialog)
from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QPixmap, QFont, QIcon, QImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from transforms3d.euler import euler2quat
from geometry_msgs.msg import PoseStamped

class RobotMonitorUI(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node
        self.bridge = CvBridge()
        self.namespace = 'robot1'  # Default robot namespace

        # Initialize the UI layout
        self.initUI()

        # Initialize ROS topics for the default robot
        self.initialize_ros_topics()
        self.waypoint_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)

    def initUI(self):
        # (UI initialization code with adjustments)
        # Load icons from the package path
        package_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '..', '..', 'share', 'multi_robot_viz', 'icons')
        self.icon_up = QIcon(os.path.join(package_path, 'up-arrow.png'))
        self.icon_down = QIcon(os.path.join(package_path, 'down-arrow.png'))
        self.icon_left = QIcon(os.path.join(package_path, 'left-arrow.png'))
        self.icon_right = QIcon(os.path.join(package_path, 'right-arrow.png'))
        self.icon_stop = QIcon(os.path.join(package_path, 'stop.png'))

        # Main layout
        main_layout = QVBoxLayout(self)

        # Create a horizontal layout for the header
        header_layout = QHBoxLayout()

        # Create the "Multiviz" logo as a header
        title_label = QLabel("MULTIVIZ")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #FF6F61; letter-spacing: 2px;")

        # Create the kill switch button
        self.kill_switch_button = QPushButton('Kill Switch')
        self.kill_switch_button.setFont(QFont("Arial", 20, QFont.Bold))
        self.kill_switch_button.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
                color: #ffffff;
                border: 2px solid #000000;
                border-radius: 15px;
                padding: 10px 20px;
                text-align: center;
            }
            QPushButton:pressed {
                background-color: #cc0000;
            }
        """)

        # Connect the button to the kill_switch method
        self.kill_switch_button.clicked.connect(self.kill_switch)

        # Add the title label and kill switch button to the header layout
        header_layout.addWidget(title_label)
        header_layout.addWidget(self.kill_switch_button)

        # Add the header layout to the main layout
        main_layout.addLayout(header_layout)

        # Add a gradient background to the main window
        self.setStyleSheet("""
            QWidget {
                background-color: #2c3e50;
            }
            QLabel {
                color: #FFFFFF;
            }
            QPushButton {
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
                background-color: #4CAF50;
                color: white;
                border: 2px solid #333;
                text-align: center;
            }
            QPushButton:pressed {
                background-color: #45a049;
                transform: scale(0.98);
            }
            QComboBox, QLineEdit {
                border: 1px solid #ccc;
                border-radius: 5px;
                padding: 5px;
                background-color: #f2f2f2;
                color: #333;
                text-align: center;
                font-weight: bold;
            }
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #76c7c0;
                width: 20px;
            }
        """)

        # Robot selection and data layout
        data_layout = QHBoxLayout()

        # Left pane for robot controls and info
        left_frame = QFrame(self)
        left_layout = QVBoxLayout(left_frame)

        # Styling font for headlines
        headline_font = QFont("Arial", 12, QFont.Bold)

        # Robot selection
        robot_label = QLabel("Select Robot:", self)
        robot_label.setFont(headline_font)
        left_layout.addWidget(robot_label)
        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(["robot1", "robot2", "robot3", "robot4"])
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        left_layout.addWidget(self.robot_dropdown)

        # Battery Status (Optional, depends on availability)
        battery_label = QLabel("Battery Status:", self)
        battery_label.setFont(headline_font)
        left_layout.addWidget(battery_label)
        self.battery_bar = QProgressBar(self)
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)  # Initial value
        left_layout.addWidget(self.battery_bar)

        # Display /cmd_vel topic with value in a box
        cmd_vel_label = QLabel("Command Velocity (cmd_vel):", self)
        cmd_vel_label.setFont(headline_font)
        left_layout.addWidget(cmd_vel_label)
        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        left_layout.addWidget(self.cmd_vel_value)

        # Display odom topic with value in a box
        odom_label = QLabel("Odometry (odom):", self)
        odom_label.setFont(headline_font)
        left_layout.addWidget(odom_label)
        self.odom_value = QLineEdit(self)
        self.odom_value.setReadOnly(True)
        left_layout.addWidget(self.odom_value)

        # Teleoperation controls
        teleop_label = QLabel("Teleop Controls:", self)
        teleop_label.setFont(headline_font)
        left_layout.addWidget(teleop_label)
        teleop_grid = QGridLayout()

        # Teleop buttons with attractive icons (Gamepad-like controls)
        self.btn_up = QPushButton('', self)
        self.btn_down = QPushButton('', self)
        self.btn_left = QPushButton('', self)
        self.btn_right = QPushButton('', self)
        self.btn_stop = QPushButton('', self)

        # Set the icons
        self.btn_up.setIcon(self.icon_up)
        self.btn_down.setIcon(self.icon_down)
        self.btn_left.setIcon(self.icon_left)
        self.btn_right.setIcon(self.icon_right)
        self.btn_stop.setIcon(self.icon_stop)

        # Set the icon sizes
        icon_size = 32
        self.btn_up.setIconSize(QSize(icon_size, icon_size))
        self.btn_down.setIconSize(QSize(icon_size, icon_size))
        self.btn_left.setIconSize(QSize(icon_size, icon_size))
        self.btn_right.setIconSize(QSize(icon_size, icon_size))
        self.btn_stop.setIconSize(QSize(icon_size, icon_size))

        # Connect buttons to movement functions
        self.btn_up.clicked.connect(self.move_forward)
        self.btn_down.clicked.connect(self.move_backward)
        self.btn_left.clicked.connect(self.turn_left)
        self.btn_right.clicked.connect(self.turn_right)
        self.btn_stop.clicked.connect(self.stop_robot)

        # Add buttons to grid layout
        teleop_grid.addWidget(self.btn_up, 0, 1)
        teleop_grid.addWidget(self.btn_left, 1, 0)
        teleop_grid.addWidget(self.btn_stop, 1, 1)
        teleop_grid.addWidget(self.btn_right, 1, 2)
        teleop_grid.addWidget(self.btn_down, 2, 1)

        left_layout.addLayout(teleop_grid)

        # Waypoints and Cartographer section
        waypoint_label = QLabel("Set Waypoints:", self)
        waypoint_label.setFont(headline_font)
        left_layout.addWidget(waypoint_label)

        self.btn_set_waypoint = QPushButton('Set Waypoint', self)
        self.btn_set_waypoint.clicked.connect(self.set_waypoint)
        left_layout.addWidget(self.btn_set_waypoint)

        # Right pane for visual data (map, camera, laser)
        right_frame = QFrame(self)
        right_layout = QVBoxLayout(right_frame)

        # Map showing robot position (Placeholder)
        map_label = QLabel("Map View (Simulated):", self)
        map_label.setFont(headline_font)
        right_layout.addWidget(map_label)
        self.map_view = QLabel(self)
        self.map_view.setPixmap(QPixmap(320, 240))
        right_layout.addWidget(self.map_view)

        # Camera image
        camera_label = QLabel("Camera View:", self)
        camera_label.setFont(headline_font)
        right_layout.addWidget(camera_label)
        self.image_label = QLabel(self)
        self.image_label.setPixmap(QPixmap(320, 240))
        right_layout.addWidget(self.image_label)

        # Laser scan data with value in a box
        laser_label = QLabel("Laser Scan Data:", self)
        laser_label.setFont(headline_font)
        right_layout.addWidget(laser_label)
        self.laser_value = QLineEdit(self)
        self.laser_value.setReadOnly(True)
        right_layout.addWidget(self.laser_value)

        # Create a splitter to separate the left and right frames
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_frame)
        splitter.addWidget(right_frame)
        splitter.setSizes([300, 500])

        # Add the splitter to the data layout
        data_layout.addWidget(splitter)
        main_layout.addLayout(data_layout)

        # Set window properties
        self.setLayout(main_layout)
        self.setWindowTitle('Multiviz - Robot Monitor and Control Interface')
        self.setGeometry(100, 100, 1280, 720)  # 16:9 ratio

        # Timer to update the battery level (simulated)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_battery)
        self.timer.start(1000)  # Update every second

    def update_battery(self):
        # Simulate battery drain (if needed)
        pass  # Implement battery status update if available

    def on_robot_selected(self, robot_name):
        self.namespace = robot_name
        # Update subscribers and publishers to use the new namespace
        self.initialize_ros_topics()

    def initialize_ros_topics(self):
        # Close existing subscriptions and publishers if they exist
        try:
            self.node.destroy_subscription(self.odom_sub)
            self.node.destroy_subscription(self.scan_sub)
            self.node.destroy_subscription(self.image_sub)
            self.node.destroy_publisher(self.cmd_vel_pub)
        except AttributeError:
            pass  # Subscriptions don't exist yet

        # Odometry subscriber
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.namespace}/odom',
            self.odom_callback,
            10)

        # Laser scan subscriber
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.scan_callback,
            10)

        # Image subscriber
        self.image_sub = self.node.create_subscription(
            Image,
            f'/{self.namespace}/camera/image_raw',
            self.image_callback,
            10)

        # Cmd_vel publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)

    def odom_callback(self, msg):
        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Update GUI elements
        self.odom_value.setText(f"Position: {position.x:.2f}, {position.y:.2f}")

    def scan_callback(self, msg):
        # Extract scan data
        self.laser_data = msg.ranges
        # Update GUI elements
        if len(self.laser_data) >= 3:
            self.laser_value.setText(f"Laser Distances: {self.laser_data[0]:.2f}, {self.laser_data[1]:.2f}, {self.laser_data[2]:.2f} ...")
        else:
            self.laser_value.setText("Laser data unavailable")

    def image_callback(self, msg):
        # Convert ROS Image message to QPixmap
        image = self.ros_image_to_qpixmap(msg)
        self.image_label.setPixmap(image)

    def ros_image_to_qpixmap(self, msg):
        # Convert ROS Image message to QPixmap
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            q_image = q_image.rgbSwapped()  # Convert BGR to RGB
            return QPixmap.fromImage(q_image)
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")
            return QPixmap()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText("Moving forward")

    def move_backward(self):
        twist = Twist()
        twist.linear.x = -0.2  # Move backward at 0.2 m/s
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText("Moving backward")

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText("Turning left")

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5  # Turn right at 0.5 rad/s
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText("Turning right")

    def stop_robot(self):
        twist = Twist()
        # Zero velocities stop the robot
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_value.setText("Stopping robot")

    def set_waypoint(self):
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y, yaw):')
        if ok:
            x,y,yaw = waypoint.split(',')
            x = float(x)
            y = float(y)
            yaw = float(yaw)
            quaternion = euler2quat(0.0, 0.0, yaw)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = quaternion[0]
            pose.pose.orientation.x = quaternion[1]
            pose.pose.orientation.y = quaternion[2]
            pose.pose.orientation.z = quaternion[3]
            self.waypoint_pub.publish(pose)
            print(f"Waypoint set to: {waypoint}")

    def kill_switch(self):
        # Perform the kill switch action
        self.node.get_logger().info("Kill switch activated!")
        self.stop_robot()
        # Optionally, disable further commands
        try:
            self.node.destroy_publisher(self.cmd_vel_pub)
        except AttributeError:
            pass
        self.cmd_vel_value.setText("Robot stopped (Kill switch activated)")

    def closeEvent(self, event):
        # Clean up ROS 2 resources
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('robot_monitor_ui')
    app = QApplication(sys.argv)
    ex = RobotMonitorUI(node)
    ex.show()

    # Start the ROS 2 spinning in a separate thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    ret = app.exec_()

    # Clean up
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
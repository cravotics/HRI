import sys
import rclpy
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton, QGridLayout, QInputDialog)
from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QPixmap, QFont, QIcon
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotMonitorUI(QWidget):
    def __init__(self):
        super().__init__()
        self.namespace = 'waffle1'  # Default robot namespace

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('robot_monitor_ui')

        # Initialize the UI layout
        self.initUI()

        # Initialize ROS topics
        self.initialize_ros_topics()

        # Start the ROS timer
        self.start_ros_timer()

    def initUI(self):
        # Load icons (adjust the paths as necessary)
        self.icon_up = QIcon('up-arrow.png')
        self.icon_down = QIcon('down-arrow.png')
        self.icon_left = QIcon('left-arrow.png')
        self.icon_right = QIcon('right-arrow.png')
        self.icon_stop = QIcon('stop.png')

        # Main layout
        main_layout = QVBoxLayout(self)

        # Header layout with title and kill switch
        header_layout = QHBoxLayout()

        title_label = QLabel("MULTIVIZ")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #FF6F61; letter-spacing: 2px;")

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
        self.kill_switch_button.clicked.connect(self.kill_switch)

        header_layout.addWidget(title_label)
        header_layout.addWidget(self.kill_switch_button)
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

        headline_font = QFont("Arial", 12, QFont.Bold)

        # Robot selection
        robot_label = QLabel("Select Robot:", self)
        robot_label.setFont(headline_font)
        left_layout.addWidget(robot_label)
        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(["waffle1", "waffle2", "waffle3"])
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        left_layout.addWidget(self.robot_dropdown)

        # Battery Status (Optional)
        battery_label = QLabel("Battery Status:", self)
        battery_label.setFont(headline_font)
        left_layout.addWidget(battery_label)
        self.battery_bar = QProgressBar(self)
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)  # Initial value
        left_layout.addWidget(self.battery_bar)

        # Command Velocity
        cmd_vel_label = QLabel("Command Velocity (cmd_vel):", self)
        cmd_vel_label.setFont(headline_font)
        left_layout.addWidget(cmd_vel_label)
        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        left_layout.addWidget(self.cmd_vel_value)

        # Odometry
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

        # Teleop buttons
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

        # Waypoints (Placeholder)
        waypoint_label = QLabel("Set Waypoints:", self)
        waypoint_label.setFont(headline_font)
        left_layout.addWidget(waypoint_label)

        self.btn_set_waypoint = QPushButton('Set Waypoint', self)
        self.btn_set_waypoint.clicked.connect(self.set_waypoint)
        left_layout.addWidget(self.btn_set_waypoint)

        # Right pane for visual data
        right_frame = QFrame(self)
        right_layout = QVBoxLayout(right_frame)

        # Map View (Placeholder)
        map_label = QLabel("Map View (Simulated):", self)
        map_label.setFont(headline_font)
        right_layout.addWidget(map_label)
        self.map_view = QLabel(self)
        self.map_view.setPixmap(QPixmap(320, 240))
        right_layout.addWidget(self.map_view)

        # Laser scan data
        laser_label = QLabel("Laser Scan Data:", self)
        laser_label.setFont(headline_font)
        right_layout.addWidget(laser_label)
        self.laser_value = QLineEdit(self)
        self.laser_value.setReadOnly(True)
        right_layout.addWidget(self.laser_value)

        # Create a splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_frame)
        splitter.addWidget(right_frame)
        splitter.setSizes([400, 800])

        # Add the splitter to the data layout
        data_layout.addWidget(splitter)
        main_layout.addLayout(data_layout)

        # Set window properties
        self.setLayout(main_layout)
        self.setWindowTitle('Multiviz - Robot Monitor and Control Interface')
        self.setGeometry(100, 100, 1280, 720)

        # Timer to update the battery level (simulated)
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(1000)  # Update every second

    def initialize_ros_topics(self):
        # Close existing subscriptions and publishers if they exist
        try:
            self.node.destroy_subscription(self.odom_sub)
            self.node.destroy_subscription(self.scan_sub)
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

        # Cmd_vel publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)

    def start_ros_timer(self):
        # Timer to periodically call rclpy.spin_once
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)  # Call every 50 milliseconds

    def spin_ros(self):
        # Process ROS events
        rclpy.spin_once(self.node, timeout_sec=0)

    # ROS Callbacks
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        text = f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n" \
               f"Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}"
        self.update_odom_ui(text)

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) >= 3:
            text = f"Distances: {ranges[0]:.2f}, {ranges[1]:.2f}, {ranges[2]:.2f}..."
        else:
            text = "Laser data unavailable"
        self.update_laser_ui(text)

    # Methods to handle commands from GUI
    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        text = f"Linear: {linear}, Angular: {angular}"
        self.update_cmd_vel_ui(text)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.update_cmd_vel_ui("Stopped")

    def kill_switch(self):
        self.node.get_logger().info("Kill switch activated!")
        self.stop_robot()
        try:
            self.node.destroy_publisher(self.cmd_vel_pub)
        except AttributeError:
            pass
        self.update_cmd_vel_ui("Robot stopped (Kill switch activated)")

    def change_namespace(self, namespace):
        self.namespace = namespace
        self.initialize_ros_topics()

    def update_battery(self):
        # Simulate battery drain
        current_value = self.battery_bar.value()
        new_value = max(0, current_value - 1)
        self.battery_bar.setValue(new_value)

    def on_robot_selected(self, robot_name):
        self.change_namespace(robot_name)

    def move_forward(self):
        self.publish_cmd_vel(0.2, 0.0)

    def move_backward(self):
        self.publish_cmd_vel(-0.2, 0.0)

    def turn_left(self):
        self.publish_cmd_vel(0.0, 0.5)

    def turn_right(self):
        self.publish_cmd_vel(0.0, -0.5)

    def set_waypoint(self):
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y):')
        if ok:
            print(f"Waypoint set to: {waypoint}")
            self.cmd_vel_value.setText(f"Moving to waypoint: {waypoint}")
            # Implement waypoint navigation logic here

    # Slots to update UI
    def update_odom_ui(self, text):
        self.odom_value.setText(text)

    def update_laser_ui(self, text):
        self.laser_value.setText(text)

    def update_cmd_vel_ui(self, text):
        self.cmd_vel_value.setText(text)

    def closeEvent(self, event):
        # Clean up ROS resources
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    ex = RobotMonitorUI()
    ex.show()

    ret = app.exec_()

    # Clean up
    sys.exit(ret)

if __name__ == '__main__':
    main()

import sys
import os
import rclpy
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton, QGridLayout, QInputDialog, QTextEdit)
from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QPixmap, QFont, QIcon, QImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat
import numpy as np
from nav_msgs.msg import OccupancyGrid
import pyttsx3
import speech_recognition as sr
from PyQt5.QtCore import Qt, QTimer, QSize, QThread, pyqtSignal
import sys
import os
sys.path.append('/home/gautamsn/HRI/src/Depth-Anything-V2')
from depth_anything_v2.dpt import DepthAnythingV2
import cv2
import torch

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vits'
model = DepthAnythingV2(**model_configs[encoder])

try:
    model.load_state_dict(torch.load('/home/gautamsn/HRI/src/gui/gui/depth_anything_v2/depth_anything_v2_vits.pth', map_location='cpu'))
except FileNotFoundError:
    print("Model file not found. Please ensure it is located in 'depth_anything_v2/' or update the path.")
    exit(1)

model = model.to(DEVICE).eval()


class VoiceRecognitionThread(QThread):
    command_recognized = pyqtSignal(str)

    def __init__(self, recognizer):
        super().__init__()
        self.recognizer = recognizer
        self.running = False

    def run(self):
        while self.running:
            try:
                with sr.Microphone() as source:
                    print("Listening for voice command...")
                    audio = self.recognizer.listen(source)
                    command = self.recognizer.recognize_google(audio)
                    print(f"Command recognized: {command}")
                    self.command_recognized.emit(command)
            except Exception as e:
                print(f"Error: {e}")

    def stop(self):
        self.running = False
        self.wait()


class RobotMonitorUI(QWidget):
    def __init__(self):
        super().__init__()
        self.namespace = 'waffle_1'  # Default robot namespace

        # List of robot names
        self.robot_names = ["waffle_1", "waffle_2", "waffle_3", "waffle_4"]
        self.robot_batteries = {}
        self.voices = []
        self.voices = self.speech_engine.getProperty('voices')

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('robot_monitor_ui')
        self.bridge = CvBridge()
        self._action_client1 = ActionClient(self.node, NavigateToPose, '/waffle_1/navigate_to_pose')
        self._action_client2 = ActionClient(self.node, NavigateToPose, '/waffle_2/navigate_to_pose')
        self._action_client3 = ActionClient(self.node, NavigateToPose, '/waffle_3/navigate_to_pose')
        self._action_client4 = ActionClient(self.node, NavigateToPose, '/waffle_4/navigate_to_pose')
        self.map_sub = self.node.create_subscription(OccupancyGrid,'/map', self.map_callback,10)

        # Initialize the UI layout
        self.initUI()

        # Initialize ROS topics
        self.initialize_ros_topics()

        # Start the ROS timer
        self.start_ros_timer()
        
        # Initialize voice recognition and speech synthesis
        self.recognizer = sr.Recognizer()
        self.speech_engine = pyttsx3.init()
        self.speech_engine.setProperty('rate', 150)  # Set speaking rate
        self.speech_engine.setProperty('volume', 1.0)  # Volume level (0.0 to 1.0)
        if len(self.voices) > 1:
            self.speech_engine.setProperty('voice', self.voices[1].id)
        else:
            print("Voice 2 not found, using default voice.")
        # Initialize the voice recognition thread
        self.voice_thread = VoiceRecognitionThread(self.recognizer)
        self.voice_thread.command_recognized.connect(self.process_command)

    def initUI(self):
        # Load icons (adjust the paths as necessary)
        self.icon_up = QIcon('up-arrow.png')
        self.icon_down = QIcon('down-arrow.png')
        self.icon_left = QIcon('left-arrow.png')
        self.icon_right = QIcon('right-arrow.png')
        self.icon_stop = QIcon('stop.png')

        # Main layout
        main_layout = QVBoxLayout(self)

        # Apply a beige gradient background
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(
                    x1: 0, y1: 0, x2: 1, y2: 1,
                    stop: 0 #f5f5dc, stop: 0.5 #e3d5b5, stop: 1 #d2b48c
                );
                font-family: Arial;
                color: #000000;
            }
            QLabel {
                font-size: 16px;
                font-weight: bold;
                text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.1);
                color: #000000;
            }
            QPushButton {
                background-color: #8b4513;
                color: #ffffff;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #ffffff;
                border-radius: 10px;
                padding: 10px 20px;
                box-shadow: 3px 3px 5px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: #5a2e0f;
                box-shadow: inset 2px 2px 5px rgba(0, 0, 0, 0.3);
            }
            QProgressBar {
                text-align: center;
                font-size: 12px;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
                color: #000000;
            }
            QLineEdit, QTextEdit {
                background: rgba(255, 255, 255, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.5);
                color: #000000;
                padding: 5px;
                border-radius: 5px;
            }
            QComboBox {
                background: rgba(255, 255, 255, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.5);
                color: #000000;
                padding: 5px;
                border-radius: 5px;
            }
            QProgressBar::chunk {
                background-color: #8b4513;
            }
        """)

        # Create a vertical layout for the header
        header_layout = QVBoxLayout()

        # Create the "Multiviz" logo as a header
        title_label = QLabel("MULTIVIZ 🤖 ")
        title_label.setFont(QFont("Helvetica", 36, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            color: #8b4513;
            text-shadow: 4px 4px 6px #000000;
            letter-spacing: 2px;
        """)

        # Add the title label to the header layout
        header_layout.addWidget(title_label)
        
        # Add a button to start/stop voice control
        self.voice_control_button = QPushButton("Start Voice Control")
        self.voice_control_button.clicked.connect(self.toggle_voice_control)
        main_layout.addWidget(self.voice_control_button)


        # Create a horizontal layout for robot batteries
        robot_battery_layout = QHBoxLayout()
        for robot_name in self.robot_names:
            robot_battery_container = QVBoxLayout()
            robot_battery_label = QLabel(robot_name)
            robot_battery_label.setAlignment(Qt.AlignCenter)
            robot_battery_label.setFont(QFont("Arial", 12, QFont.Bold))
            robot_battery_bar = QProgressBar()
            robot_battery_bar.setRange(0, 100)
            robot_battery_bar.setValue(100)
            robot_battery_bar.setTextVisible(True)
            robot_battery_bar.setFixedWidth(100)
            # Store the battery bar in self.robot_batteries[robot_name] for updating later
            self.robot_batteries[robot_name] = robot_battery_bar
            # Add label and bar to container
            robot_battery_container.addWidget(robot_battery_label)
            robot_battery_container.addWidget(robot_battery_bar)
            # Add container to robot_battery_layout
            robot_battery_layout.addLayout(robot_battery_container)
        # Add robot battery layout to header layout
        header_layout.addLayout(robot_battery_layout)

        # Create the kill switch button
        self.kill_switch_button = QPushButton('Kill Switch')
        self.kill_switch_button.setFont(QFont("Arial", 20, QFont.Bold))
        self.kill_switch_button.setStyleSheet("""
            QPushButton {
                background-color: #8b0000;
                color: #ffffff;
                border: 2px solid #ffffff;
                border-radius: 15px;
                padding: 10px 20px;
                text-align: center;
                text-shadow: 2px 2px 4px #000;
            }
            QPushButton:pressed {
                background-color: #5a0000;
            }
        """)

        # Connect the button to the kill_switch method
        self.kill_switch_button.clicked.connect(self.kill_switch)

        # Add the kill switch button to the header layout
        header_layout.addWidget(self.kill_switch_button)

        # Add the header layout to the main layout
        main_layout.addLayout(header_layout)

        # Robot selection layout
        robot_selection_layout = QHBoxLayout()
        robot_label = QLabel("Select Robot  🤖 ", self)
        robot_label.setFont(QFont("Arial", 12, QFont.Bold))
        robot_selection_layout.addWidget(robot_label)
        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(self.robot_names)
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        robot_selection_layout.addWidget(self.robot_dropdown)
        main_layout.addLayout(robot_selection_layout)

        # Data layout (contains the splitter)
        data_layout = QHBoxLayout()

        # Left pane for robot controls and info
        left_frame = QFrame(self)
        left_layout = QVBoxLayout(left_frame)

        headline_font = QFont("Arial", 14, QFont.Bold)

        # Command Velocity
        cmd_vel_label = QLabel("Command Velocity", self)
        cmd_vel_label.setFont(headline_font)
        left_layout.addWidget(cmd_vel_label)
        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        left_layout.addWidget(self.cmd_vel_value)

        # Odometry
        odom_label = QLabel("Odometry 🧭", self)
        odom_label.setFont(headline_font)
        
        left_layout.addWidget(odom_label)
        self.odom_value = QLineEdit(self)
        self.odom_value.setReadOnly(True)
        left_layout.addWidget(self.odom_value)

        # Teleoperation controls
        teleop_label = QLabel("Teleop Controls 🕹️", self)
        teleop_label.setFont(headline_font)
        left_layout.addWidget(teleop_label)
        teleop_grid = QGridLayout()

        # Teleop buttons
        self.btn_up = QPushButton('⬆️')
        self.btn_down = QPushButton('⬇️')
        self.btn_left = QPushButton('⬅️')
        self.btn_right = QPushButton('➡️')
        self.btn_stop = QPushButton('🛑')

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

        # Logs section
        logs_label = QLabel("Logs 📚", self)
        logs_label.setFont(headline_font)
        left_layout.addWidget(logs_label)
        self.logs_text_edit = QTextEdit(self)
        self.logs_text_edit.setReadOnly(True)
        left_layout.addWidget(self.logs_text_edit)

        # Waypoints
        waypoint_label = QLabel("Set Waypoints 📍", self)
        waypoint_label.setFont(headline_font)
        left_layout.addWidget(waypoint_label)

        self.btn_set_waypoint = QPushButton('Set Waypoint', self)
        self.btn_set_waypoint.clicked.connect(self.set_waypoint)
        left_layout.addWidget(self.btn_set_waypoint)

        left_layout.addStretch()  # Add stretch to fill space

        # Right pane for visual data
        right_frame = QFrame(self)
        right_layout = QVBoxLayout(right_frame)

        # Experimentation Display View (Placeholder)
        map_label = QLabel("Experimentaion Display:", self)
        map_label.setFont(headline_font)
        right_layout.addWidget(map_label)
        self.map_view = QLabel(self)
        self.map_view.setPixmap(QPixmap(640, 240))  
        self.map_view.setScaledContents(True)
        right_layout.addWidget(self.map_view)

        # Image View (Camera)
        image_label = QLabel("Camera View 📹 ", self)
        image_label.setFont(headline_font)
        right_layout.addWidget(image_label)
        self.image_view = QLabel(self)
        self.image_view.setPixmap(QPixmap(640, 240))
        self.image_view.setScaledContents(True)
        right_layout.addWidget(self.image_view)

        # Laser scan data
        laser_label = QLabel("Laser Scan Data", self)
        laser_label.setFont(headline_font)
        right_layout.addWidget(laser_label)
        self.laser_value = QLineEdit(self)
        self.laser_value.setReadOnly(True)
        right_layout.addWidget(self.laser_value)

        right_layout.addStretch()  # Add stretch to fill space

        # Create a splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_frame)
        splitter.addWidget(right_frame)
        # Set stretch factors to control resizing behavior
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

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
    
    def map_callback(self, msg):
        try:
            map_image = self.process_map_data(msg)
            self.map_view.setPixmap(map_image)
        except Exception as e:
            self.node.get_logger().error(f"Failed to update map: {e}")
            
    def process_map_data(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        data[data == -1] = 128 
        data[data == 0] = 255 
        data[data > 0] = 0 

        q_image = QImage(data.data, width, height, width, QImage.Format_Grayscale8)

        return QPixmap.fromImage(q_image)

    def image_callback(self, msg):
        pixmap = self.ros_image_to_qpixmap(msg)
        depth_pixmap = self.ros_image_to_depth_qpixmap(msg)
        if pixmap:
            self.image_view.setPixmap(pixmap)
        if depth_pixmap:
            self.map_view.setPixmap(depth_pixmap)

    def ros_image_to_qpixmap(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            return pixmap
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
            return None

    def ros_image_to_depth_qpixmap(self, msg):
        try:
            # Convert ROS image to OpenCV format (from ROS Image message to OpenCV image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Pass the OpenCV image to the model for depth inference
            depth_image = model.infer_image(cv_image)
            
            # Normalize depth values to the 0-255 range and convert to uint8
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = depth_image_normalized.astype(np.uint8)
            
            # Apply a color map to the normalized depth image
            depth_colormap = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            
            # Convert BGR to RGB for Qt compatibility
            depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)
            
            # Get image dimensions
            height, width, channel = depth_colormap.shape
            bytes_per_line = 3 * width
            
            # Create QImage from depth_colormap
            q_image = QImage(depth_colormap.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Create QPixmap from QImage for display
            pixmap = QPixmap.fromImage(q_image)
            return pixmap

        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
            return None


    # Methods to handle commands from GUI
    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        text = f"Linear: {linear}, Angular: {angular}"
        self.update_cmd_vel_ui(text)
        # Log the command
        # self.logs_text_edit.append(f"{self.namespace} cmd_vel: {text}")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.update_cmd_vel_ui("Stopped")
        self.cmd_vel_value.setText("Stopping robot")
        self.speech_engine.say("Stopping robot")
        self.speech_engine.runAndWait()

    def kill_switch(self):
        self.node.get_logger().info("Kill switch activated!")
        self.stop_robot()
        try:
            self.node.destroy_publisher(self.cmd_vel_pub)
        except AttributeError:
            pass
        self.update_cmd_vel_ui("Robot stopped (Kill switch activated)")
        self.logs_text_edit.append("Kill switch activated!")

    def toggle_voice_control(self):
        if self.voice_thread.isRunning():
            self.voice_thread.stop()
            self.voice_control_button.setText("Start Voice Control 🗣️")
        else:
            self.voice_thread.running = True
            self.voice_thread.start()
            self.voice_control_button.setText("Stop Voice Control")

    def process_command(self, command):
        print(f"Processing command: {command}")
        command_handlers = {
            "move forward": self.move_forward,
            "move backward": self.move_backward,
            "kill all": self.kill_switch,
            "activate  kill switch": self.kill_switch,
            "turn left":self.turn_left,
            "turn right":self.turn_right,
            "stop robot":self.stop_robot,
            "move straight": self.move_forward,
            "go back": self.move_backward,
            "kill robots": self.kill_switch,
            "kill robot": self.kill_switch,
            "stop robots":self.stop_robot,
        }
        
        handler = command_handlers.get(command)
        if handler:
            handler()
            self.speech_engine.runAndWait()
        else:
            print("Unknown command")
            self.speech_engine.say("Unknown command")
            self.speech_engine.runAndWait()


    def change_namespace(self, namespace):
        self.namespace = namespace
        self.initialize_ros_topics()
        self.logs_text_edit.append(f"Switched to {namespace}")

    def update_battery(self):
        # Simulate battery drain for all robots
        for robot_name in self.robot_names:
            battery_bar = self.robot_batteries[robot_name]
            current_value = battery_bar.value()
            new_value = max(0, current_value - 1)
            battery_bar.setValue(new_value)
            # Change battery bar color based on level
            if new_value < 30:
                battery_bar.setStyleSheet("""
                    QProgressBar::chunk {
                        background-color: red;
                    }
                """)
            else:
                battery_bar.setStyleSheet("""
                    QProgressBar::chunk {
                        background-color: green;
                    }
                """)

    def on_robot_selected(self, robot_name):
        self.change_namespace(robot_name)

    def move_forward(self):
        self.speech_engine.say("Moving forward")
        self.speech_engine.runAndWait()
        self.cmd_vel_value.setText("Moving forward")
        self.publish_cmd_vel(0.2, 0.0)
        self.logs_text_edit.append(f"{self.namespace} moving forward")

    def move_backward(self):
        self.speech_engine.say("Moving backward")
        self.speech_engine.runAndWait()
        self.cmd_vel_value.setText("Moving backward")
        self.publish_cmd_vel(-0.2, 0.0)
        self.logs_text_edit.append(f"{self.namespace} moving backward")

    def turn_left(self):
        self.publish_cmd_vel(0.0, 0.5)
        self.logs_text_edit.append(f"{self.namespace} turning left")
        self.cmd_vel_value.setText("Turning left")
        self.speech_engine.say("Turning left")
        self.speech_engine.runAndWait()

    def turn_right(self):
        self.publish_cmd_vel(0.0, -0.5)
        self.logs_text_edit.append(f"{self.namespace} turning right")
        self.cmd_vel_value.setText("Turning right")
        self.speech_engine.say("Turning right")
        self.speech_engine.runAndWait()

    def set_waypoint(self):
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y, yaw):')
        if ok:
            x,y,yaw = waypoint.split(',')
            x = float(x)
            y = float(y)
            yaw = float(yaw)
            print(x,y,yaw)
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
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            if self.namespace == 'waffle_1':
                self._action_client1.send_goal_async(goal_msg)
            elif self.namespace == 'waffle_2':
                self._action_client2.send_goal_async(goal_msg)
            elif self.namespace == 'waffle_3':
                self._action_client3.send_goal_async(goal_msg)
            elif self.namespace == 'waffle_4':
                self._action_client4.send_goal_async(goal_msg)
            

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

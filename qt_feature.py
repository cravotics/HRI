# qt_features.py
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QFrame, QLineEdit, QPushButton, QGridLayout,
                             QInputDialog, QTextEdit, QSplitter)
from PyQt5.QtCore import Qt, QSize, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon, QPixmap
from PyQt5.QtWidgets import QMessageBox

from PyQt5.QtWidgets import QInputDialog
from ros_integration import QuaternionConverter  # Import Quaternion conversion utility
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import pyttsx3
import speech_recognition as sr


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
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface  # Injected ROS node instance
        self.robot_names = self.ros_interface.namespace_list
        self.namespace = self.robot_names[0]
        self.robot_batteries = {}
        
        self.initUI()
        
        # Initialize voice recognition and speech synthesis.
        self.recognizer = sr.Recognizer()
        self.speech_engine = pyttsx3.init()
        self.voices = self.speech_engine.getProperty('voices')
        self.speech_engine.setProperty('rate', 150)
        self.speech_engine.setProperty('volume', 1.0)
        if len(self.voices) > 1:
            self.speech_engine.setProperty('voice', self.voices[1].id)
        else:
            print("Voice 2 not found, using default voice.")
        self.voice_thread = VoiceRecognitionThread(self.recognizer)
        self.voice_thread.command_recognized.connect(self.process_command)

    def initUI(self):
        # Load icons.
        self.icon_up = QIcon('up-arrow.png')
        self.icon_down = QIcon('down-arrow.png')
        self.icon_left = QIcon('left-arrow.png')
        self.icon_right = QIcon('right-arrow.png')
        self.icon_stop = QIcon('stop.png')
        
        main_layout = QVBoxLayout(self)
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1,
                    stop: 0 #f5f5dc, stop: 0.5 #e3d5b5, stop: 1 #d2b48c);
                font-family: Arial;
                color: #000000;
            }
            QLabel { font-size: 16px; font-weight: bold; }
            QPushButton {
                background-color: #8b4513;
                color: #ffffff;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #ffffff;
                border-radius: 10px;
                padding: 10px 20px;
            }
            QPushButton:pressed { background-color: #5a2e0f; }
            QProgressBar {
                text-align: center;
                font-size: 12px;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
            }
            QLineEdit, QTextEdit, QComboBox {
                background: rgba(255, 255, 255, 0.1);
                border: 1px solid rgba(255, 255, 255, 0.5);
                padding: 5px;
                border-radius: 5px;
            }
            QProgressBar::chunk { background-color: #8b4513; }
        """)
        # Header section.
        header_layout = QVBoxLayout()
        title_label = QLabel("MULTIVIZ 🤖 ")
        title_label.setFont(QFont("Helvetica", 36, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title_label)
        self.voice_control_button = QPushButton("Start Voice Control")
        self.voice_control_button.clicked.connect(self.toggle_voice_control)
        main_layout.addWidget(self.voice_control_button)
        
        # Robot battery status.
        robot_battery_layout = QHBoxLayout()
        for robot_name in self.robot_names:
            container = QVBoxLayout()
            label = QLabel(robot_name)
            label.setAlignment(Qt.AlignCenter)
            label.setFont(QFont("Arial", 12, QFont.Bold))
            battery_bar = QProgressBar()
            battery_bar.setRange(0, 100)
            battery_bar.setValue(100)
            battery_bar.setTextVisible(True)
            battery_bar.setFixedWidth(100)
            self.robot_batteries[robot_name] = battery_bar
            container.addWidget(label)
            container.addWidget(battery_bar)
            robot_battery_layout.addLayout(container)
        header_layout.addLayout(robot_battery_layout)
        
        # Kill switch button.
        self.kill_switch_button = QPushButton('Kill Switch')
        self.kill_switch_button.setFont(QFont("Arial", 20, QFont.Bold))
        self.kill_switch_button.setStyleSheet("""
            QPushButton {
                background-color: #8b0000;
                color: #ffffff;
                border: 2px solid #ffffff;
                border-radius: 15px;
                padding: 10px 20px;
            }
            QPushButton:pressed { background-color: #5a0000; }
        """)
        self.kill_switch_button.clicked.connect(self.kill_switch)
        header_layout.addWidget(self.kill_switch_button)
        main_layout.addLayout(header_layout)
        
        # Robot selection dropdown.
        robot_selection_layout = QHBoxLayout()
        robot_label = QLabel("Select Robot 🤖")
        robot_label.setFont(QFont("Arial", 12, QFont.Bold))
        robot_selection_layout.addWidget(robot_label)
        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(self.robot_names)
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        robot_selection_layout.addWidget(self.robot_dropdown)
        main_layout.addLayout(robot_selection_layout)
        
        # Data layout: Left pane (controls/info) and Right pane (visual displays).
        data_layout = QHBoxLayout()
        left_frame = QFrame(self)
        left_layout = QVBoxLayout(left_frame)
        headline_font = QFont("Arial", 12, QFont.Bold)
        # Command velocity display.
        cmd_vel_label = QLabel("Command Velocity")
        cmd_vel_label.setFont(headline_font)
        left_layout.addWidget(cmd_vel_label)
        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        left_layout.addWidget(self.cmd_vel_value)
        # Odometry display.
        odom_label = QLabel("Odometry 🧭")
        odom_label.setFont(headline_font)
        left_layout.addWidget(odom_label)
        self.odom_value = QLineEdit(self)
        self.odom_value.setReadOnly(True)
        left_layout.addWidget(self.odom_value)
        # Teleoperation controls.
        teleop_label = QLabel("Teleop Controls 🕹️")
        teleop_label.setFont(headline_font)
        left_layout.addWidget(teleop_label)
        teleop_grid = QGridLayout()
        self.btn_up = QPushButton('⬆️')
        self.btn_down = QPushButton('⬇️')
        self.btn_left = QPushButton('⬅️')
        self.btn_right = QPushButton('➡️')
        self.btn_stop = QPushButton('🛑')
        self.btn_up.setIcon(self.icon_up)
        self.btn_down.setIcon(self.icon_down)
        self.btn_left.setIcon(self.icon_left)
        self.btn_right.setIcon(self.icon_right)
        self.btn_stop.setIcon(self.icon_stop)
        icon_size = 32
        for btn in [self.btn_up, self.btn_down, self.btn_left, self.btn_right, self.btn_stop]:
            btn.setIconSize(QSize(icon_size, icon_size))
        self.btn_up.clicked.connect(self.move_forward)
        self.btn_down.clicked.connect(self.move_backward)
        self.btn_left.clicked.connect(self.turn_left)
        self.btn_right.clicked.connect(self.turn_right)
        self.btn_stop.clicked.connect(self.stop_robot)
        teleop_grid.addWidget(self.btn_up, 0, 1)
        teleop_grid.addWidget(self.btn_left, 1, 0)
        teleop_grid.addWidget(self.btn_stop, 1, 1)
        teleop_grid.addWidget(self.btn_right, 1, 2)
        teleop_grid.addWidget(self.btn_down, 2, 1)
        left_layout.addLayout(teleop_grid)
        # Logs section.
        logs_label = QLabel("Logs 📚")
        logs_label.setFont(headline_font)
        left_layout.addWidget(logs_label)
        self.logs_text_edit = QTextEdit(self)
        self.logs_text_edit.setReadOnly(True)
        left_layout.addWidget(self.logs_text_edit)
        # Waypoint input.
        waypoint_label = QLabel("Set Waypoints 📍")
        waypoint_label.setFont(headline_font)
        left_layout.addWidget(waypoint_label)
        self.btn_set_waypoint = QPushButton('Set Waypoint', self)
        self.btn_set_waypoint.clicked.connect(self.set_waypoint)
        left_layout.addWidget(self.btn_set_waypoint)
        left_layout.addStretch()
        # Right pane: Map, Camera, and Laser data.
        right_frame = QFrame(self)
        right_layout = QVBoxLayout(right_frame)
        map_label = QLabel("Experimentation Display:")
        map_label.setFont(headline_font)
        right_layout.addWidget(map_label)
        self.map_view = QLabel(self)
        self.map_view.setPixmap(QPixmap(640, 240))
        self.map_view.setScaledContents(True)
        right_layout.addWidget(self.map_view)
        image_label = QLabel("Camera View 📹")
        image_label.setFont(headline_font)
        right_layout.addWidget(image_label)
        self.image_view = QLabel(self)
        self.image_view.setPixmap(QPixmap(640, 240))
        self.image_view.setScaledContents(True)
        right_layout.addWidget(self.image_view)
        laser_label = QLabel("Laser Scan Data")
        laser_label.setFont(headline_font)
        right_layout.addWidget(laser_label)
        self.laser_value = QLineEdit(self)
        self.laser_value.setReadOnly(True)
        right_layout.addWidget(self.laser_value)
        right_layout.addStretch()
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_frame)
        splitter.addWidget(right_frame)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        data_layout.addWidget(splitter)
        main_layout.addLayout(data_layout)
        self.setLayout(main_layout)
        self.setWindowTitle('Multiviz - Robot Monitor and Control Interface')
        self.setGeometry(100, 100, 1280, 720)
        # Timer to update battery levels (simulated).
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(20000)
    
    # --- UI Callback Methods ---
    def on_robot_selected(self, robot_name):
        self.namespace = robot_name
        self.logs_text_edit.append(f"Switched to {robot_name}")
    
    def publish_cmd_vel(self, linear, angular):
        try:
            idx = self.ros_interface.namespace_list.index(self.namespace)
            self.ros_interface.publish_cmd_vel_to_robot(idx, linear, angular)
            self.update_cmd_vel_ui(f"Linear: {linear}, Angular: {angular}")
        except ValueError:
            self.logs_text_edit.append(f"Namespace {self.namespace} not found.")
    
    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)
        self.update_cmd_vel_ui("Stopped")
        self.cmd_vel_value.setText("Stopping robot")
        self.speech_engine.say("Stopping robot")
        self.speech_engine.runAndWait()
    
    def kill_switch(self):
        self.ros_interface.get_logger().info("Kill switch activated!")
        self.stop_robot()
        try:
            idx = self.ros_interface.namespace_list.index(self.namespace)
            self.ros_interface.cmd_vel_pubs[idx].destroy()
        except Exception as e:
            self.logs_text_edit.append(str(e))
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
            "move straight": self.move_forward,
            "move backward": self.move_backward,
            "go back": self.move_backward,
            "kill all": self.kill_switch,
            "kill robots": self.kill_switch,
            "kill robot": self.kill_switch,
            "activate kill switch": self.kill_switch,
            "turn left": self.turn_left,
            "turn right": self.turn_right,
            "stop robot": self.stop_robot,
            "stop robots": self.stop_robot,
        }
        handler = command_handlers.get(command)
        if handler:
            handler()
            self.speech_engine.runAndWait()
        else:
            print("Unknown command")
            self.speech_engine.say("Unknown command")
            self.speech_engine.runAndWait()
    
    def update_cmd_vel_ui(self, text):
        self.cmd_vel_value.setText(text)
    
    def update_battery(self):
        for robot_name in self.robot_names:
            battery_bar = self.robot_batteries[robot_name]
            current_value = battery_bar.value()
            new_value = max(0, current_value - 1)
            battery_bar.setValue(new_value)
            if new_value < 30:
                battery_bar.setStyleSheet("QProgressBar::chunk { background-color: red; }")
            else:
                battery_bar.setStyleSheet("QProgressBar::chunk { background-color: green; }")
    
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
        """
        Open a dialog box to collect waypoint coordinates and send them as a goal to the selected robot.
        """
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y, yaw):')

        if ok:
            try:
                # Parsing the  input
                x, y, yaw = map(float, waypoint.split(','))

                # this will call RosIntegration function to send goal
                idx = self.ros_interface.namespace_list.index(self.namespace)
                goal_msg = self.ros_interface.create_nav_goal(x, y, yaw)

                # Call the send_navigation_goal() function
                send_goal_future = self.ros_interface.send_navigation_goal(idx, goal_msg)

                # Notify the user
                if send_goal_future is not None:
                    self.logs_text_edit.append(f"Waypoint set for {self.namespace} at ({x}, {y}, {yaw})")
                else:
                    QMessageBox.critical(self, "Error", "Failed to send navigation goal.")

            except ValueError:
                QMessageBox.warning(self, "Input Error", "Please enter valid numeric values for x, y, and yaw.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Unexpected error: {e}")
    
    def closeEvent(self, event):
        self.ros_interface.destroy_node()
        from rclpy import shutdown
        shutdown()
        event.accept()

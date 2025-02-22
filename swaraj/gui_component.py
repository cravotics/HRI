import sys
import os
import yaml
import pyttsx3
import speech_recognition as sr
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar,
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton,
                             QGridLayout, QInputDialog, QTextEdit, QDesktopWidget, QScrollArea)
from PyQt5.QtCore import Qt, QSize, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QFont, QIcon
from ros_component import RosInterface

def load_robot_names(filename="parameters.yaml"):
    """Load robot namespaces from a YAML file using an absolute path."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(base_dir, filename)
    try:
        with open(filepath, "r") as file:
            data = yaml.safe_load(file)
            print("Loaded YAML data:", data)  # Debug print to verify YAML content
            return [robot["namespace"] for robot in data["robots"]]
    except Exception as e:
        print(f"Error loading YAML file from {filepath}: {e}")
        return ["waffle_1"]  # Fallback default

# Voice recognition thread
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

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        # Initialize speech engine and recognizer
        self.recognizer = sr.Recognizer()
        self.speech_engine = pyttsx3.init()
        self.speech_engine.setProperty('rate', 150)
        self.speech_engine.setProperty('volume', 1.0)
        voices = self.speech_engine.getProperty('voices')
        if len(voices) > 1:
            self.speech_engine.setProperty('voice', voices[1].id)
        else:
            print("Voice 2 not found, using default voice.")
        
        # Load robot names dynamically from YAML
        self.robot_names = load_robot_names()
        self.robot_batteries = {}
        self.kill_act = False

        self.initUI()

        # Create the ROS interface passing in UI update callbacks
        self.ros_interface = RosInterface(
            odom_callback=self.update_odom_ui,
            scan_callback=self.update_laser_ui,
            image_callback=self.update_image_ui,
            depth_callback=self.update_map_ui,  # Depth and occupancy map share the same widget
            occupancy_map_callback=self.update_map_ui,
            cmd_vel_callback=self.update_cmd_vel_ui
        )

        self.voice_thread = VoiceRecognitionThread(self.recognizer)
        self.voice_thread.command_recognized.connect(self.process_command)

        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(2000)

    def initUI(self):
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)

        self.icon_up = QIcon('up-arrow.png')
        self.icon_down = QIcon('down-arrow.png')
        self.icon_left = QIcon('left-arrow.png')
        self.icon_right = QIcon('right-arrow.png')
        self.icon_stop = QIcon('stop.png')

        screen = QDesktopWidget().screenGeometry()
        width, height = screen.width(), screen.height()
        self.setFixedSize(int(width * 0.8), int(height * 0.8))

        main_layout = QVBoxLayout(self)
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

        header_layout = QVBoxLayout()
        title_label = QLabel("MULTIVIZ 🤖 ")
        title_label.setFont(QFont("Helvetica", 36, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            color: #8b4513;
            text-shadow: 4px 4px 6px #000000;
            letter-spacing: 2px;
        """)
        header_layout.addWidget(title_label)

        self.voice_control_button = QPushButton("Start Voice Control")
        self.voice_control_button.clicked.connect(self.toggle_voice_control)
        main_layout.addWidget(self.voice_control_button)

        robot_battery_layout = QHBoxLayout()
        for robot_name in self.robot_names:
            battery_container = QVBoxLayout()
            battery_label = QLabel(robot_name)
            battery_label.setAlignment(Qt.AlignCenter)
            battery_label.setFont(QFont("Arial", 12, QFont.Bold))
            battery_bar = QProgressBar()
            battery_bar.setRange(0, 100)
            battery_bar.setValue(100)
            battery_bar.setTextVisible(True)
            battery_bar.setFixedWidth(100)
            self.robot_batteries[robot_name] = battery_bar
            battery_container.addWidget(battery_label)
            battery_container.addWidget(battery_bar)
            robot_battery_layout.addLayout(battery_container)
        header_layout.addLayout(robot_battery_layout)

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
        self.kill_switch_button.clicked.connect(self.kill_switch)
        header_layout.addWidget(self.kill_switch_button)

        main_layout.addLayout(header_layout)

        robot_selection_layout = QHBoxLayout()
        robot_label = QLabel("Select Robot  🤖 ", self)
        robot_label.setFont(QFont("Arial", 12, QFont.Bold))
        robot_selection_layout.addWidget(robot_label)
        self.robot_dropdown = QComboBox(self)
        self.robot_dropdown.addItems(self.robot_names)
        self.robot_dropdown.currentTextChanged.connect(self.on_robot_selected)
        robot_selection_layout.addWidget(self.robot_dropdown)
        main_layout.addLayout(robot_selection_layout)

        data_layout = QHBoxLayout()
        left_frame = QFrame(self)
        left_layout = QVBoxLayout(left_frame)
        headline_font = QFont("Arial", 14, QFont.Bold)

        cmd_vel_label = QLabel("Command Velocity", self)
        cmd_vel_label.setFont(headline_font)
        left_layout.addWidget(cmd_vel_label)
        self.cmd_vel_value = QLineEdit(self)
        self.cmd_vel_value.setReadOnly(True)
        left_layout.addWidget(self.cmd_vel_value)

        odom_label = QLabel("Odometry 🧭", self)
        odom_label.setFont(headline_font)
        left_layout.addWidget(odom_label)
        self.odom_value = QLineEdit(self)
        self.odom_value.setReadOnly(True)
        left_layout.addWidget(self.odom_value)

        teleop_label = QLabel("Teleop Controls 🕹️", self)
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
        icon_size = int(height * 0.05)
        self.btn_up.setIconSize(QSize(icon_size, icon_size))
        self.btn_down.setIconSize(QSize(icon_size, icon_size))
        self.btn_left.setIconSize(QSize(icon_size, icon_size))
        self.btn_right.setIconSize(QSize(icon_size, icon_size))
        self.btn_stop.setIconSize(QSize(icon_size, icon_size))
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

        logs_label = QLabel("Logs 📚", self)
        logs_label.setFont(headline_font)
        left_layout.addWidget(logs_label)
        self.logs_text_edit = QTextEdit(self)
        self.logs_text_edit.setReadOnly(True)
        left_layout.addWidget(self.logs_text_edit)

        waypoint_label = QLabel("Set Waypoints 📍", self)
        waypoint_label.setFont(headline_font)
        left_layout.addWidget(waypoint_label)
        self.btn_set_waypoint = QPushButton('Set Waypoint', self)
        self.btn_set_waypoint.clicked.connect(self.set_waypoint)
        left_layout.addWidget(self.btn_set_waypoint)
        left_layout.addStretch()

        right_frame = QFrame(self)
        right_layout = QVBoxLayout(right_frame)
        map_label = QLabel("Experimentaion Display:", self)
        map_label.setFont(headline_font)
        right_layout.addWidget(map_label)
        self.map_view = QLabel(self)
        self.map_view.setPixmap(QPixmap(320, 120))
        self.map_view.setScaledContents(True)
        right_layout.addWidget(self.map_view)
        image_label = QLabel("Camera View 📹 ", self)
        image_label.setFont(headline_font)
        right_layout.addWidget(image_label)
        self.image_view = QLabel(self)
        self.image_view.setPixmap(QPixmap(320, 120))
        self.image_view.setScaledContents(True)
        right_layout.addWidget(self.image_view)
        laser_label = QLabel("Laser Scan Data", self)
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
        self.setFixedSize(1024, 768)

    # ----------------------------
    # UI Update Callback Methods
    # ----------------------------
    def update_odom_ui(self, text):
        self.odom_value.setText(text)

    def update_laser_ui(self, text):
        self.laser_value.setText(text)

    def update_cmd_vel_ui(self, text):
        self.cmd_vel_value.setText(text)

    def update_image_ui(self, pixmap):
        self.image_view.setPixmap(pixmap)

    def update_map_ui(self, pixmap):
        self.map_view.setPixmap(pixmap)

    # ----------------------------
    # Button/Command Handlers
    # ----------------------------
    def move_forward(self):
        if not self.kill_act:
            self.speech_engine.say("Moving forward")
            self.speech_engine.runAndWait()
            self.ros_interface.move_forward()
            self.logs_text_edit.append(f"{self.robot_dropdown.currentText()} moving forward")

    def move_backward(self):
        if not self.kill_act:
            self.speech_engine.say("Moving backward")
            self.speech_engine.runAndWait()
            self.ros_interface.move_backward()
            self.logs_text_edit.append(f"{self.robot_dropdown.currentText()} moving backward")

    def turn_left(self):
        if not self.kill_act:
            self.ros_interface.turn_left()
            self.logs_text_edit.append(f"{self.robot_dropdown.currentText()} turning left")
            self.speech_engine.say("Turning left")
            self.speech_engine.runAndWait()

    def turn_right(self):
        if not self.kill_act:
            self.ros_interface.turn_right()
            self.logs_text_edit.append(f"{self.robot_dropdown.currentText()} turning right")
            self.speech_engine.say("Turning right")
            self.speech_engine.runAndWait()

    def stop_robot(self):
        self.ros_interface.stop_robot()
        self.update_cmd_vel_ui("Stopped")
        self.logs_text_edit.append("Robot stopping")
        self.speech_engine.say("Stopping robot")
        self.speech_engine.runAndWait()

    def kill_switch(self):
        self.ros_interface.kill_switch(self.robot_names)
        self.logs_text_edit.append("Kill switch activated!")
        self.kill_act = True

    def set_waypoint(self):
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y, yaw):')
        if ok and not self.kill_act:
            self.ros_interface.set_waypoint(waypoint)

    def on_robot_selected(self, robot_name):
        self.ros_interface.change_namespace(robot_name)
        self.logs_text_edit.append(f"Switched to {robot_name}")

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
            "activate kill switch": self.kill_switch,
            "turn left": self.turn_left,
            "turn right": self.turn_right,
            "stop robot": self.stop_robot,
            "move straight": self.move_forward,
            "go back": self.move_backward,
            "kill robots": self.kill_switch,
            "kill robot": self.kill_switch,
            "stop robots": self.stop_robot,
        }
        handler = command_handlers.get(command.lower())
        if handler:
            handler()
            self.speech_engine.runAndWait()
        else:
            print("Unknown command")
            self.speech_engine.say("Unknown command")
            self.speech_engine.runAndWait()

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

    def closeEvent(self, event):
        self.ros_interface.shutdown()
        event.accept()

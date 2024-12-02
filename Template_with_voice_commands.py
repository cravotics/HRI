import sys
import random
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QProgressBar, 
                             QHBoxLayout, QSplitter, QFrame, QLineEdit, QPushButton, QGridLayout, QInputDialog)
from PyQt5.QtCore import Qt, QTimer, QSize, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QFont, QIcon
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
    def __init__(self):
        super().__init__()

        # Initialize the simulated data
        self.battery_level = 100
        self.cmd_vel = [0.0, 0.0]
        self.odom = [0.0, 0.0, 0.0]
        self.laser_data = [0.0] * 360  # Simulating 360 degrees of laser data
        self.image = QPixmap(320, 240)
        self.image.fill(Qt.black)

        # Initialize the UI layout
        self.initUI()

        # Timer for simulating sensor data updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)  # Update every second

        # Initialize voice recognition and speech synthesis
        self.recognizer = sr.Recognizer()
        self.speech_engine = pyttsx3.init()
        self.speech_engine.setProperty('rate', 150)  # Set speaking rate

        # Initialize the voice recognition thread
        self.voice_thread = VoiceRecognitionThread(self.recognizer)
        self.voice_thread.command_recognized.connect(self.process_command)


    def initUI(self):
        # Main layout
        main_layout = QVBoxLayout(self)

        # Create a horizontal layout for the header
        header_layout = QHBoxLayout()

        # Create the "Multiviz" logo as a header
        title_label = QLabel("MULTI⦿VIZ")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #FF6F61; text-shadow: 4px 4px 6px #000000; letter-spacing: 2px;")

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
                text-shadow: 2px 2px 4px #000;
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
        
        # Add a button to start/stop voice control
        self.voice_control_button = QPushButton("Start Voice Control")
        self.voice_control_button.clicked.connect(self.toggle_voice_control)
        main_layout.addWidget(self.voice_control_button)


        # Add a gradient background to the main window
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #2c3e50, stop:1 #3498db);
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
                text-shadow: 2px 2px 4px #000;
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
        self.robot_dropdown.addItems(["Robot1", "Robot2", "Robot3"])
        left_layout.addWidget(self.robot_dropdown)

        # Battery Status
        battery_label = QLabel("Battery Status:", self)
        battery_label.setFont(headline_font)
        left_layout.addWidget(battery_label)
        self.battery_bar = QProgressBar(self)
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(self.battery_level)
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

        # Set the icon paths (Ensure these icons are in the same folder or provide correct paths)
        self.btn_up.setIcon(QIcon('up-arrow.png'))
        self.btn_down.setIcon(QIcon('down-arrow.png'))
        self.btn_left.setIcon(QIcon('left-arrow.png'))
        self.btn_right.setIcon(QIcon('right-arrow.png'))
        self.btn_stop.setIcon(QIcon('stop.png'))

        # Set the icon sizes
        icon_size = 64
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

        # Map showing robot position
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
        self.image_label.setPixmap(self.image)
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

    def toggle_voice_control(self):
        if self.voice_thread.isRunning():
            self.voice_thread.stop()
            self.voice_control_button.setText("Start Voice Control")
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
        }
        
        handler = command_handlers.get(command)
        if handler:
            handler()
            self.speech_engine.runAndWait()
        else:
            print("Unknown command")
            self.speech_engine.say("Unknown command")
            self.speech_engine.runAndWait()
        

    def update_data(self):
        # Simulate battery drain
        self.battery_level = max(0, self.battery_level - random.uniform(0.5, 2.0))
        self.battery_bar.setValue(int(self.battery_level))

        # Simulate cmd_vel topic
        self.cmd_vel = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        self.cmd_vel_value.setText(f"Linear: {self.cmd_vel[0]:.2f}, Angular: {self.cmd_vel[1]:.2f}")

        # Simulate odom topic
        self.odom = [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-3.14, 3.14)]
        self.odom_value.setText(f"Position: {self.odom[0]:.2f}, {self.odom[1]:.2f}, Orientation: {self.odom[2]:.2f}")

        # Simulate laser/scan data
        self.laser_data = [random.uniform(0.1, 10.0) for _ in range(360)]
        self.laser_value.setText(f"Laser Distances: {self.laser_data[0:5]} ...")

        # Simulate camera image update
        self.image.fill(Qt.darkGray if random.random() > 0.5 else Qt.black)
        self.image_label.setPixmap(self.image)

    # Teleoperation functions
    def move_forward(self):
        print("Moving forward")
        self.speech_engine.say("Moving forward")
        self.speech_engine.runAndWait()
        self.cmd_vel_value.setText("Moving forward")

    def move_backward(self):
        print("Moving backward")
        self.cmd_vel_value.setText("Moving backward")

    def turn_left(self):
        print("Turning left")
        self.cmd_vel_value.setText("Turning left")
        self.speech_engine.say("Turning left")
        self.speech_engine.runAndWait()

    def turn_right(self):
        print("Turning right")
        self.cmd_vel_value.setText("Turning right")
        self.speech_engine.say("Turning right")
        self.speech_engine.runAndWait()

    def stop_robot(self):
        print("Stopping robot")
        self.cmd_vel_value.setText("Stopping robot")
        self.speech_engine.say("Stopping robot")
        self.speech_engine.runAndWait()


    # Set waypoint using Cartographer
    def set_waypoint(self):
        waypoint, ok = QInputDialog.getText(self, 'Set Waypoint', 'Enter waypoint coordinates (x, y):')
        if ok:
            print(f"Waypoint set to: {waypoint}")
            self.cmd_vel_value.setText(f"Moving to waypoint: {waypoint}")

    # Kill switch function
    def kill_switch(self):
        # Perform the kill switch action
        print("Kill switch activated!")
        self.cmd_vel = [0.0, 0.0]  # Stop the robot's movement
        self.cmd_vel_value.setText("Robot stopped (Kill switch activated)")
        # Optionally, stop any ongoing actions, timers, etc.
        # For simulation purposes, we can reset the battery level
        self.battery_level = 0
        self.battery_bar.setValue(int(self.battery_level))
        # Stop the update timer
        self.timer.stop()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotMonitorUI()
    ex.show()
    sys.exit(app.exec_())

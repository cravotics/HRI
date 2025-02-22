import os
import rclpy
import yaml
import numpy as np
import cv2
import torch
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from transforms3d.euler import euler2quat
from PyQt5.QtCore import QTimer

# Set up device and model configurations
DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vits'
from depth_anything_v2.dpt import DepthAnythingV2  # Import your model class
model = DepthAnythingV2(**model_configs[encoder])
try:
    model.load_state_dict(torch.load('/home/swaraj/HRI/src/gui/gui/depth_anything_v2/depth_anything_v2_vits.pth',
                                     map_location='cpu', weights_only=True))
except FileNotFoundError:
    print("Model file not found. Please ensure it is located in 'depth_anything_v2/' or update the path.")
    exit(1)
model = model.to(DEVICE).eval()

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
        return ["waffle_1"]

class RosInterface:
    """
    Encapsulates all ROS functionality. It creates the ROS node, sets up subscriptions,
    publishers, action clients, and conversion methods. UI callback functions can be provided
    so that ROS callbacks update the GUI.
    """
    def __init__(self,
                 odom_callback=None,
                 scan_callback=None,
                 image_callback=None,
                 depth_callback=None,
                 occupancy_map_callback=None,
                 cmd_vel_callback=None):
        self.odom_callback_ui = odom_callback
        self.scan_callback_ui = scan_callback
        self.image_callback_ui = image_callback       # For RGB camera image
        self.depth_callback_ui = depth_callback         # For depth image
        self.occupancy_map_callback_ui = occupancy_map_callback  # For occupancy grid map
        self.cmd_vel_callback_ui = cmd_vel_callback

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('robot_monitor_ui')
        self.bridge = CvBridge()
        
        # Load robot namespaces dynamically from YAML
        self.robot_names = load_robot_names()
        self.namespace = self.robot_names[0]  # Default to the first namespace

        # Create dynamic action clients for each namespace
        self.action_clients = {
            ns: ActionClient(self.node, NavigateToPose, f'/{ns}/navigate_to_pose')
            for ns in self.robot_names
        }

        self.initialize_ros_topics()
        
        # Occupancy grid map subscription (remains static)
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Create QTimer to periodically spin ROS events
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)

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
            10)
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.scan_callback,
            10)
        self.image_sub = self.node.create_subscription(
            Image,
            f'/{self.namespace}/camera/image_raw',
            self.image_callback,
            10)
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10)

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    # ----------------------------
    # ROS Callback Methods
    # ----------------------------
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        text = (f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n"
                f"Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, "
                f"z={orientation.z:.2f}, w={orientation.w:.2f}")
        if self.odom_callback_ui:
            self.odom_callback_ui(text)

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) >= 3:
            text = f"Distances: {ranges[0]:.2f}, {ranges[1]:.2f}, {ranges[2]:.2f}..."
        else:
            text = "Laser data unavailable"
        if self.scan_callback_ui:
            self.scan_callback_ui(text)

    def map_callback(self, msg):
        try:
            map_image = self.process_map_data(msg)
            if self.occupancy_map_callback_ui:
                self.occupancy_map_callback_ui(map_image)
        except Exception as e:
            self.node.get_logger().error(f"Failed to update map: {e}")

    def image_callback(self, msg):
        pixmap = self.ros_image_to_qpixmap(msg)
        depth_pixmap = self.ros_image_to_depth_qpixmap(msg)
        if pixmap and self.image_callback_ui:
            self.image_callback_ui(pixmap)
        if depth_pixmap and self.depth_callback_ui:
            self.depth_callback_ui(depth_pixmap)

    # ----------------------------
    # Helper Conversion Methods
    # ----------------------------
    def ros_image_to_qpixmap(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            from PyQt5.QtGui import QImage, QPixmap
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            return QPixmap.fromImage(q_image)
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
            return None

    def ros_image_to_depth_qpixmap(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth_image = model.infer_image(cv_image)
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = depth_image_normalized.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)
            height, width, channel = depth_colormap.shape
            bytes_per_line = 3 * width
            from PyQt5.QtGui import QImage, QPixmap
            q_image = QImage(depth_colormap.data, width, height, bytes_per_line, QImage.Format_RGB888)
            return QPixmap.fromImage(q_image)
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
            return None

    def process_map_data(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        data[data == -1] = 128 
        data[data == 0] = 255 
        data[data > 0] = 0 
        from PyQt5.QtGui import QImage, QPixmap
        q_image = QImage(data.data, width, height, width, QImage.Format_Grayscale8)
        return QPixmap.fromImage(q_image)

    # ----------------------------
    # Command Methods
    # ----------------------------
    def publish_cmd_vel(self, linear, angular):
        try:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)
            text = f"Linear: {linear}, Angular: {angular}"
            if self.cmd_vel_callback_ui:
                self.cmd_vel_callback_ui(text)
        except Exception as e:
            self.node.get_logger().error(f"Publish error: {e}")

    def stop_robot(self):
        try:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            if self.cmd_vel_callback_ui:
                self.cmd_vel_callback_ui("Stopped")
        except Exception as e:
            self.node.get_logger().error(f"Stop error: {e}")

    def move_forward(self):
        self.publish_cmd_vel(0.2, 0.0)

    def move_backward(self):
        self.publish_cmd_vel(-0.2, 0.0)

    def turn_left(self):
        self.publish_cmd_vel(0.0, 0.5)

    def turn_right(self):
        self.publish_cmd_vel(0.0, -0.5)

    def kill_switch(self, robot_names):
        current_namespace = self.namespace
        for ns in robot_names:
            self.change_namespace(ns)
            self.stop_robot()
        self.change_namespace(current_namespace)
        self.stop_robot()
        try:
            self.node.destroy_publisher(self.cmd_vel_pub)
        except Exception as e:
            self.node.get_logger().error(f"Destroy publisher error: {e}")
        try:
            for client in self.action_clients.values():
                client.destroy()
        except Exception as e:
            self.node.get_logger().error(f"Destroy action client error: {e}")
        try:
            self.node.destroy_subscription(self.odom_sub)
            self.node.destroy_subscription(self.scan_sub)
            self.node.destroy_subscription(self.image_sub)
        except Exception as e:
            self.node.get_logger().error(f"Destroy subscription error: {e}")
        if self.cmd_vel_callback_ui:
            self.cmd_vel_callback_ui("Robot stopped (Kill switch activated)")

    def change_namespace(self, namespace):
        self.namespace = namespace
        self.initialize_ros_topics()

    def set_waypoint(self, waypoint_str):
        try:
            x_str, y_str, yaw_str = waypoint_str.split(',')
            x = float(x_str)
            y = float(y_str)
            yaw = float(yaw_str)
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
            # Use the dynamic action client based on current namespace
            client = self.action_clients.get(self.namespace)
            if client:
                client.send_goal_async(goal_msg)
            else:
                self.node.get_logger().error(f"No action client found for namespace {self.namespace}")
        except Exception as e:
            self.node.get_logger().error(f"Waypoint error: {e}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()

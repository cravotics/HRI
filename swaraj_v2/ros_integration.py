# ros_integration.py
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from sensor_msgs.msg import LaserScan, Image
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap

class QuaternionConverter:
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion (w, x, y, z order)
        :param roll: Rotation around x-axis in radians
        :param pitch: Rotation around y-axis in radians
        :param yaw: Rotation around z-axis in radians
        :return: geometry_msgs.msg.Quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    @staticmethod
    def quaternion_to_euler(q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        :param q: geometry_msgs.msg.Quaternion
        :return: tuple (roll, pitch, yaw) in radians
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

class RosIntegration(Node):
    def __init__(self, namespace_list=["robot_1", "robot_2"]):
        super().__init__('robot_monitor_ui')
        self.namespace_list = namespace_list
        self.bridge = CvBridge()
        self.nav_clients = []
        self._init_ros_components()

    def _init_ros_components(self):
        self.odom_subs = []
        self.scan_subs = []
        self.image_subs = []
        self.cmd_vel_pubs = []
        
        for ns in self.namespace_list:
            # Subscribers
            self.odom_subs.append(self.create_subscription(
                Odometry, f'/{ns}/odom', self.odom_callback, 10))
            
            self.scan_subs.append(self.create_subscription(
                LaserScan, f'/{ns}/scan', self.scan_callback, 10))
            
            self.image_subs.append(self.create_subscription(
                Image, f'/{ns}/camera/image_raw', self.image_callback, 10))
            
            # Publishers
            self.cmd_vel_pubs.append(self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10))
            
            # Action Clients
            self.nav_clients.append(ActionClient(
                self, NavigateToPose, f'/{ns}/navigate_to_pose'))

    def create_nav_goal(self, x, y, theta, frame_id='map'):
        """
        Create NavigateToPose goal message
        :param x: Target x position
        :param y: Target y position
        :param theta: Target orientation (yaw) in radians
        :param frame_id: Reference frame
        :return: NavigateToPose.Goal
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = QuaternionConverter.euler_to_quaternion(0, 0, theta)
        
        return goal_msg

    def send_navigation_goal(self, robot_index, goal_msg, feedback_callback=None):
        """
        Send navigation goal to specified robot
        :param robot_index: Index in namespace_list
        :param goal_msg: NavigateToPose.Goal message
        :param feedback_callback: Optional feedback callback function
        :return: Future result
        """
        if robot_index < 0 or robot_index >= len(self.nav_clients):
            self.get_logger().error("Invalid robot index")
            return None

        client = self.nav_clients[robot_index]
        
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return None

        send_goal_future = client.send_goal_async(
            goal_msg, 
            feedback_callback=feedback_callback
        )
        
        return send_goal_future

    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        text = (f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n"
                f"Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, "
                f"z={orientation.z:.2f}, w={orientation.w:.2f}")
        self.get_logger().info(text)

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) >= 3:
            text = f"Distances: {ranges[0]:.2f}, {ranges[1]:.2f}, {ranges[2]:.2f}..."
        else:
            text = "Laser data unavailable"
        self.get_logger().info(text)

    def image_callback(self, msg):
        pixmap = self.ros_image_to_qpixmap(msg)
        if pixmap:
            # Optionally, forward the pixmap to the UI via a signal.
            pass

    def ros_image_to_qpixmap(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            return QPixmap.fromImage(q_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return None

    def publish_cmd_vel_to_robot(self, index, linear, angular):
        """Publish a Twist message to the robot at the given index."""
        if index < 0 or index >= len(self.cmd_vel_pubs):
            self.get_logger().error("Invalid robot index for cmd_vel publisher")
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pubs[index].publish(twist)

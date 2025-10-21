import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np

class BumpAndGo(Node):
    def __init__(self):
        super().__init__('bump_and_go_node')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('min_distance_threshold', 0.6)

        # Read parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

        # Internal states
        self.scan_data = None
        self.current_yaw = 0.0
        self.state = 'GO_FORWARD'
        self.turn_direction = 1  # 1 = left, -1 = right

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def laser_callback(self, msg):
        self.scan_data = msg

    def control_loop(self):
        if self.scan_data is None:
            return

        # Read front laser ranges (±15°)
        front_angles = np.concatenate((
            self.scan_data.ranges[0:30],
            self.scan_data.ranges[-30:]
        ))
        front_distances = [r for r in front_angles if not np.isinf(r)]
        min_front = min(front_distances) if front_distances else float('inf')

        twist = Twist()

        if self.state == 'GO_FORWARD':
            if min_front < self.min_distance_threshold:
                self.get_logger().info('Obstacle detected! Switching to ROTATE')
                self.state = 'ROTATE'
                self.turn_direction = self.choose_turn_direction()
            else:
                twist.linear.x = self.linear_speed

        elif self.state == 'ROTATE':
            # Check if front is clear
            if min_front > self.min_distance_threshold + 0.1:
                self.get_logger().info('Path clear. Switching to GO_FORWARD')
                self.state = 'GO_FORWARD'
            else:
                twist.angular.z = self.turn_direction * self.angular_speed

        self.cmd_vel_pub.publish(twist)

    def choose_turn_direction(self):
        # Simple method: compare left and right average distances
        ranges = np.array(self.scan_data.ranges)
        left = ranges[60:100]
        right = ranges[260:300]

        left_avg = np.mean([r for r in left if not np.isinf(r)] or [float('inf')])
        right_avg = np.mean([r for r in right if not np.isinf(r)] or [float('inf')])

        return 1 if left_avg > right_avg else -1

def main(args=None):
    rclpy.init(args=args)
    node = BumpAndGo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


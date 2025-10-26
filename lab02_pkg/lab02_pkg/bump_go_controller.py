import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

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
        self.position = Point()
        self.goal_pos = Pose()
        self.goal_pos.position.x = 6.5
        self.goal_pos.position.y = 3.0

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw
        self.position = msg.pose.pose.position

    def laser_callback(self, msg):
        self.scan_data = msg

    def normalize_angle(self, angle):
        """ Normalize angle to be within [-pi, pi] """
        return np.arctan2(np.sin(angle), np.cos(angle))

    def control_loop(self):
        if self.scan_data is None:
            return

        # Read front laser ranges (±10° for narrower detection)
        ### FIX: narrower detection
        front_indices = list(range(10)) + list(range(len(self.scan_data.ranges) - 10, len(self.scan_data.ranges)))
        valid_front = [(i, self.scan_data.ranges[i]) for i in front_indices if not np.isinf(self.scan_data.ranges[i])]
        min_index, min_front = min(valid_front, key=lambda x: x[1]) if valid_front else (None, float('inf'))
        min_angle = self.scan_data.angle_min + min_index * self.scan_data.angle_increment if min_index is not None else None

        # Check if goal is reached
        distance_to_goal = np.sqrt(
            (self.position.x - self.goal_pos.position.x) ** 2 +
            (self.position.y - self.goal_pos.position.y) ** 2
        )
        if distance_to_goal < 0.7 and distance_to_goal > 0.4:
            self.state = 'GOAL'
            self.get_logger().info('Goal detected')
        elif distance_to_goal <= 0.4:
            self.get_logger().info('At goal position! Stopping robot.')
            self.state = 'STOP'

        twist = Twist()

        # Dynamic obstacle threshold (smaller near goal)
        ### FIX
        if distance_to_goal < 1.0:
            dynamic_threshold = 0.3
        else:
            dynamic_threshold = self.min_distance_threshold

        # --- STATE MACHINE ---
        if self.state == 'GO_FORWARD':
            if min_front < dynamic_threshold:
                self.get_logger().info('Obstacle detected! Switching to ROTATE')
                self.state = 'ROTATE'
                self.turn_direction = self.choose_turn_direction()
            else:
                twist.linear.x = self.linear_speed

        elif self.state == 'ROTATE':
            # Check if front is clear
            if min_front > dynamic_threshold + 0.1:
                self.get_logger().info('Path clear. Switching to GO_FORWARD')
                self.state = 'GO_FORWARD'
            else:
                twist.angular.z = self.turn_direction * self.angular_speed

        elif self.state == 'GOAL':
            # FIX: Corrected self.position reference
            dinamic_threshold = 0
            goal_dx = self.goal_pos.position.x - self.position.x
            goal_dy = self.goal_pos.position.y - self.position.y
            goal_angle = np.arctan2(goal_dy, goal_dx)
            angle_error = self.normalize_angle(goal_angle - self.current_yaw)

            # Ignore obstacle avoidance here
            ### FIX
            if distance_to_goal > 0.4:
                if abs(angle_error) > np.deg2rad(15):
                    twist.angular.z = np.sign(angle_error) * (self.angular_speed * 0.5)
                    twist.linear.x = 0.05  # move slightly forward while turning
                else:
                    twist.linear.x = self.linear_speed * 0.5
            else:
                self.state = 'STOP'
                self.get_logger().info('Approaching goal, preparing to stop.')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Robot stopped at goal position.')

        

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


if __name__ == '__main__':
    main()

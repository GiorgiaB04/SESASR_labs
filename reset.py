import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from math import sqrt

class ResetNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/pose',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Bool,'/reset', 10)
        self.subscription  # prevent unused variable warning
        self.reset=Bool()
        self.reset.data=False
        self.n=0

    def listener_callback(self, msg):
        
        self.n+=1
        self.get_logger().info('I heard: "%s"' % msg.position)
        if sqrt(msg.position.x**2+msg.position.y**2)>6:
            self.reset.data=True
            
            self.publisher_.publish(self.reset)
            self.get_logger().info('Reset sent')
        else:
            []
        self.reset.data=False
        self.publisher_.publish(self.reset)
        
        


def main(args=None):
    rclpy.init(args=args)

    reset_node = ResetNode()

    rclpy.spin(reset_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
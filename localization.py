import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist

class LocalizationNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.subscription  # prevent unused variable warning
        self.position=Pose()
        self.n=0

    def listener_callback(self, msg):
        msgpos=Pose()
        self.n+=1
        self.get_logger().info('I heard: "%s"' % msg.linear)
        self.position.position.x+=msg.linear.x
        self.position.position.y+=msg.linear.y
        msgpos.position.x=self.position.position.x
        msgpos.position.y=self.position.position.y
        self.publisher_.publish(msgpos)
        self.get_logger().info(f'Publishing: x={msgpos.position.x}, y={msgpos.position.y}')
        
        


def main(args=None):
    rclpy.init(args=args)

    localizaion_node = LocalizationNode()

    rclpy.spin(localizaion_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localizaion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
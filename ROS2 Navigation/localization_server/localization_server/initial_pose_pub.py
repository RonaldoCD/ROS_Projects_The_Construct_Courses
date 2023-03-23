import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile

class InitialPosePubNode(Node):
    
    def __init__(self):
        super().__init__("initial_pose_pub")
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.subscriber_ = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.subscriber_callback,
            10)
        self.subscriber_

    def subscriber_callback(self, msg):
        pub_msg = PoseWithCovarianceStamped()
        pub_msg.pose.position.x = msg.point.x
        pub_msg.pose.position.y = msg.point.y
        pub_msg.header.frame_id = '/map'
        self.get_logger().info('Initial position set to: ("%s", "%s")' % (str(msg.point.x), str(msg.point.y)))

        self.publisher_.publish(pub_msg)

def main(args = None):
    rclpy.init(args=args)
    initial_pose_pub_node = InitialPosePubNode()
    rclpy.spin(initial_pose_pub_node)
    initial_pose_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

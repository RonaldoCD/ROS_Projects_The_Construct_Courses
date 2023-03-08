import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class TopicsQuizNode(Node):
    
    def __init__(self):
        super().__init__("topics_quiz_node")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_subscriber_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.pos_robot_x = 0
        self.pos_robot_y = 0
        self.orientation_robot_z = 0

        self.first_path_finished = False
        self.turn_finished = False

    def timer_callback(self):
        msg = Twist()
        if self.first_path_finished is False:
            if self.pos_robot_x > 0.95:
                msg.linear.x = 0.0
                self.first_path_finished = True
            else:
                msg.linear.x = 0.5
        elif self.turn_finished is False:
            if self.orientation_robot_z > 1.5025:
                msg.angular.z = 0.0
                self.turn_finished = True
            else:
                msg.angular.z = 0.3
                print("Yaw: ", self.orientation_robot_z)
        else:
            if self.pos_robot_y > 0.95:
                msg.linear.x = 0.0
            else:
                msg.linear.x = 0.5

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

    def odom_subscriber_callback(self, msg):
        self.get_logger().info('I receive: "%s"' % str(msg.pose.pose.orientation))
        self.pos_robot_x = msg.pose.pose.position.x
        self.pos_robot_y = msg.pose.pose.position.y
        quaternion = [0, 0, 0, 0]
        quaternion[0] = msg.pose.pose.orientation.x
        quaternion[1] = msg.pose.pose.orientation.y
        quaternion[2] = msg.pose.pose.orientation.z
        quaternion[3] = msg.pose.pose.orientation.w
        _, _, yaw = self.euler_from_quaternion(quaternion)
        
        self.orientation_robot_z = yaw

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args = None):
    rclpy.init(args=args)
    topics_quiz_node = TopicsQuizNode()
    rclpy.spin(topics_quiz_node)
    topics_quiz_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


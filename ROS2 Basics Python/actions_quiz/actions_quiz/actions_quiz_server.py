import time
import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import ReliabilityPolicy, QoSProfile

from actions_quiz_msg.action import Distance
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ActionQuizServer(Node):

    def __init__(self):
        super().__init__('action_quiz_server')
        self.group1 = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(self, 
            Distance, 
            'distance_as', 
            self.execute_callback,
            callback_group=self.group1) 

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_publisher = self.create_publisher(Float64, '/total_distance', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)
        
        self.start_pos_x = 0.0
        self.start_pos_y = 0.0
        self.start_pos_set = False

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

    def odom_callback(self, msg):
        if self.start_pos_set is False:
            self.start_pos_x = msg.pose.pose.position.x
            self.start_pos_y = msg.pose.pose.position.y
            self.start_pos_set = True
        else:
            self.current_pos_x = msg.pose.pose.position.x
            self.current_pos_y = msg.pose.pose.position.y
    
    def get_distance_traveled(self):
        dist_traveled = np.sqrt((self.current_pos_x - self.start_pos_x)**2 + (self.current_pos_y - self.start_pos_y)**2)
        return dist_traveled

    def execute_callback(self, goal_handle):
        cmd_msg = Twist()
        dist_msg = Float64()

        self.get_logger().info('Executing goal...')

        feedback_msg = Distance.Feedback()
        
        for i in range(goal_handle.request.seconds):
            cmd_msg.linear.x = 0.3
            self.cmd_vel_publisher.publish(cmd_msg)

            distance_traveled = self.get_distance_traveled()
            feedback_msg.current_dist = distance_traveled
            dist_msg.data = distance_traveled

            goal_handle.publish_feedback(feedback_msg)
            self.distance_publisher.publish(dist_msg)

            self.get_logger().info('Current distance traveled: "%s" meters' %str(distance_traveled))

            time.sleep(1)
            self.get_logger().info('"%s" seconds have passed' %str(i + 1))

        goal_handle.succeed()

        cmd_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_msg)

        total_distance = self.get_distance_traveled()
        result = Distance.Result()
        result.status = True
        result.total_dist = total_distance
        self.get_logger().info('The action has finished')
        self.get_logger().info('The robot has traveled: "%s" meters' %str(total_distance))
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server_node = ActionQuizServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
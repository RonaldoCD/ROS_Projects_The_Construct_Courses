import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped

class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('action_quiz_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.point_subscriber = self.create_subscription(PointStamped, 'clicked_point', self.subscriber_callback, 10)  

    def subscriber_callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))
        
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        position = [x, y, z]
        orientation = [0.0, 0.0, 0.0, 1.0]
        frame_id = "map"

        self.send_goal(frame_id, position, orientation)
        

    def send_goal(self, frame_id, position, orientation):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.pose.position.x = position[0]
        goal_msg.pose.pose.position.y = position[1]
        goal_msg.pose.pose.position.z = position[2]

        goal_msg.pose.pose.orientation.x = orientation[0]
        goal_msg.pose.pose.orientation.y = orientation[1]
        goal_msg.pose.pose.orientation.z = orientation[2]
        goal_msg.pose.pose.orientation.w = orientation[3]

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('The robot has reached its destination')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        current_x = feedback_msg.feedback.current_pose.pose.position.x
        current_y = feedback_msg.feedback.current_pose.pose.position.y
        navigation_time = feedback_msg.feedback.navigation_time.sec
        estimated_timer_reamining = feedback_msg.feedback.estimated_time_remaining.sec
        self.get_logger().info('Feedback received')
        self.get_logger().info('Current position: ("%f", "%f") ------- Nav. Time: "%d" ------- Estimated time remaining: "%d"' %(current_x, current_y, navigation_time, estimated_timer_reamining))


def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseClient()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
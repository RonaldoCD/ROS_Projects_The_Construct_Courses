import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions_quiz_msg.action import Distance


class ActionQuizClient(Node):

    def __init__(self):
        super().__init__('action_quiz_client')
        self._action_client = ActionClient(self, Distance, 'distance_as')

    def send_goal(self, seconds):
        goal_msg = Distance.Goal()
        goal_msg.seconds = seconds

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

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
        result = future.result().result
        status = result.status
        total_dist = result.total_dist
        self.get_logger().info('The robot has traveled: "%s" meters' %str(total_dist))
        self.get_logger().info('Result: {0}'.format(status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback received')
        self.get_logger().info('Current distance traveled: "%s" meters' %str(feedback.current_dist))


def main(args=None):
    rclpy.init(args=args)

    action_client = ActionQuizClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
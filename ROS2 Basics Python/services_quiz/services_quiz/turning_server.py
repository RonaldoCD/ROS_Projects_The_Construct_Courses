# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces_service interface
from services_quiz_srv.srv import Turn
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node

class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('turn_server')
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(Turn, 'turn', self.turn_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0
        

    def turn_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response

        # create a Twist message
        msg = Twist()
        
        direction = request.direction
        angular_velocity = request.angular_velocity
        turn_time = request.time

        start_t = self.get_current_time_seconds()
        
        if direction == "left":
            msg.angular.z = angular_velocity
            message_print = "Turning to left direction"
        
        else:
            msg.angular.z = - angular_velocity
            message_print = "Turning to right direction"
        
        while self.get_current_time_seconds() - start_t < turn_time:
            self.cmd_vel_publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info(message_print)
            # response state
        
        msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)
        response.success = True
        return response

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
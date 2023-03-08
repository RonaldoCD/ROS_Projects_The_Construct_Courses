#include <iostream>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include <memory>

using namespace std;
using Spin = services_quiz_srv::srv::Spin;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::placeholders::_2;

class SpinServerNode : public rclcpp::Node
{
public:
  SpinServerNode()
  : Node("spin_server_node")
  {

    spin_server = create_service<Spin>("rotate", std::bind(&SpinServerNode::server_callback, this, _1, _2));
    vel_publisher = this->create_publisher<Twist>("cmd_vel", 10);

  }

private:
  rclcpp::Service<Spin>::SharedPtr spin_server;
  rclcpp::Publisher<Twist>::SharedPtr vel_publisher;

  void server_callback(
      const std::shared_ptr<Spin::Request> request,
      const std::shared_ptr<Spin::Response> response) 
    {
        auto message = Twist();
        std::string direction = request->direction; 
        float angular_velocity = request->angular_velocity;
        int time = request->time;

        int turn_sign = 1;
        if (direction == "right"){
            turn_sign = -1;
        }
        message.angular.z = turn_sign * angular_velocity;
        int time_passed = 0;
        while (time_passed < time){
            vel_publisher->publish(message);
            std::this_thread::sleep_for(1000ms);;
            time_passed++;
        }
        message.angular.z = 0.0;
        vel_publisher->publish(message);
        response->success = true;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpinServerNode>());
    rclcpp::shutdown();
  return 0;
}
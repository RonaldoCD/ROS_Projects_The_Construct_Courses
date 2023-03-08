#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class TopicsQuizNode : public rclcpp::Node
{
public:
    TopicsQuizNode()
    : Node("topics_quiz_node")
    {
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TopicsQuizNode::scan_subs_callback, this, _1));
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer = this->create_wall_timer(500ms, std::bind(&TopicsQuizNode::timer_callback, this));
        print_timer = this->create_wall_timer(500ms, std::bind(&TopicsQuizNode::print_timer_callback, this));
    }

private:
    void scan_subs_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int ranges_size = msg->ranges.size();
        int idx_offset = ranges_size * 20 / 180;

        // RCLCPP_INFO(this->get_logger(), "Ranges size = '%d'", ranges_size);
        int front_idx = (ranges_size - 1)/ 2; 
        left_scan = msg->ranges[idx_offset];
        right_scan = msg->ranges[ranges_size - 1 - idx_offset];
        front_scan = msg->ranges[front_idx];
    }

    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();

        if (front_scan > 1.0){
            message.linear.x = linear_vel;
            message.angular.z = 0.0;
        }
        else{
            message.linear.x = 0.0;
            message.angular.z = angular_vel;
        }

        if (right_scan < 1.0){
            message.linear.x = 0.0;
            message.angular.z = angular_vel;
        }
        else if (left_scan < 1.0) {
            message.linear.x = 0.0;
            message.angular.z = - angular_vel;
        }

        vel_publisher->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Linear = '%f' and Angular = '%f'", message.linear.x, message.angular.z);
    }

    void print_timer_callback(){
        RCLCPP_INFO(this->get_logger(), "(Left, Front, Right) = ('%f', '%f', '%f')", left_scan, front_scan, right_scan);
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr print_timer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    float right_scan;
    float front_scan;
    float left_scan;
    float linear_vel = 0.3;
    float angular_vel = 0.3;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicsQuizNode>());
    rclcpp::shutdown();
    return 0;
}
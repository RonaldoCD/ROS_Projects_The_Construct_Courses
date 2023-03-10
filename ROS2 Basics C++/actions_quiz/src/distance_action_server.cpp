#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_quiz_msg/action/distance.hpp"

#include "std_msgs/msg/detail/float64__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std;


class DistanceServer : public rclcpp::Node
{
    public:
    using DistanceAct = actions_quiz_msg::action::Distance;
    using GoalHandleDistance = rclcpp_action::ServerGoalHandle<DistanceAct>;

    explicit DistanceServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("distance_action_server", options)
    {
        using namespace std::placeholders;

        odom_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        action_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = odom_callback_group;

        this->action_server_ = rclcpp_action::create_server<DistanceAct>(
        this,
        "distance_as",
        std::bind(&DistanceServer::handle_goal, this, _1, _2),
        std::bind(&DistanceServer::handle_cancel, this, _1),
        std::bind(&DistanceServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        action_callback_group);

        total_distance_publisher = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DistanceServer::odom_callback, this, _1), options1);

        current_pos_x = 0.0;
        current_pos_y = 0.0;
        start_pos_x = 0.0;
        start_pos_y = 0.0;
        start_pos_set = false;
    }

    private:
    rclcpp_action::Server<DistanceAct>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_distance_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    float current_pos_x;
    float current_pos_y;
    float start_pos_x;
    float start_pos_y;
    bool start_pos_set;

    rclcpp::CallbackGroup::SharedPtr odom_callback_group;
    rclcpp::CallbackGroup::SharedPtr action_callback_group;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DistanceAct::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDistance> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&DistanceServer::execute, this, _1), goal_handle}.detach();
    }

    float get_distance_traveled(){
        float distance_traveled;
        distance_traveled = sqrt(pow(this->current_pos_x - this->start_pos_x, 2) + pow(this->current_pos_y - this->start_pos_y, 2));
        return distance_traveled;
    }
    void execute(const std::shared_ptr<GoalHandleDistance> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DistanceAct::Feedback>();
        auto & current_distance_feedback = feedback->current_dist;
        // message = "Starting movement...";
        auto result = std::make_shared<DistanceAct::Result>();
        auto distance_msg = std_msgs::msg::Float64();
        rclcpp::Rate loop_rate(1);
        float distance_traveled = 0.0;

        for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->status = true;
                result->total_dist = distance_traveled;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // Move robot forward and send feedback
            loop_rate.sleep();
            distance_traveled = get_distance_traveled();
            current_distance_feedback = distance_traveled;
            distance_msg.data = distance_traveled;

            total_distance_publisher->publish(distance_msg);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->status = true;
            result->total_dist = distance_traveled;
            // total_distance_publisher->publish(distance_msg);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!this->start_pos_set){
            this->start_pos_x = msg->pose.pose.position.x;
            this->start_pos_y = msg->pose.pose.position.y;
            this->start_pos_set = true;
        }
        else {
            this->current_pos_x = msg->pose.pose.position.x;
            this->current_pos_y = msg->pose.pose.position.y; 
        }
    }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<DistanceServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
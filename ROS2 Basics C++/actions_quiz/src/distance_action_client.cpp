#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DistanceClient : public rclcpp::Node
{
    public:
    using DistanceAct = actions_quiz_msg::action::Distance;
    using GoalHandleDistance = rclcpp_action::ClientGoalHandle<DistanceAct>;

    explicit DistanceClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("distance_action_client", node_options), goal_done_(false)
    {
        this->client_ptr_ = rclcpp_action::create_client<DistanceAct>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "distance_as");

        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DistanceClient::send_goal, this));
    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        this->goal_done_ = false;

        if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
        }

        auto goal_msg = DistanceAct::Goal();
        goal_msg.seconds = 20;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<DistanceAct>::SendGoalOptions();
                    
        send_goal_options.goal_response_callback =
        std::bind(&DistanceClient::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
        std::bind(&DistanceClient::feedback_callback, this, _1, _2);

        send_goal_options.result_callback =
        std::bind(&DistanceClient::result_callback, this, _1);
        
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    private:
    rclcpp_action::Client<DistanceAct>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(const GoalHandleDistance::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleDistance::SharedPtr,
        const std::shared_ptr<const DistanceAct::Feedback> feedback)
    {
        RCLCPP_INFO(
        this->get_logger(), "Feedback received: %f", feedback->current_dist);
    }

    void result_callback(const GoalHandleDistance::WrappedResult & result)
    {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->status);
        RCLCPP_INFO(this->get_logger(), "Total distance received: %f", result.result->total_dist);

    }
};  // class MyActionClient

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<DistanceClient>();
        
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_client);

    while (!action_client->is_goal_done()) {
        executor.spin();
    }

    rclcpp::shutdown();
    return 0;
}
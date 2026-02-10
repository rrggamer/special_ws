#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace custom_action_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    // 1. Declare the parameter so the Node knows to look for it
    this->declare_parameter<int>("order", 10);

    auto timer_callback_lambda = [this](){ return this->send_goal(); };
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
  }

  void send_goal()
  {
    using namespace std::placeholders;
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();

    // 2. Get the value. This value comes from 'parameter_overrides' in main()
    int order_param = this->get_parameter("order").as_int();
    goal_msg.order = order_param;

    RCLCPP_INFO(this->get_logger(), "Sending goal with order: %d", order_param);

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    
    // --- Callback 1: Goal Response ---
    send_goal_options.goal_response_callback = [this](const GoalHandleFibonacci::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    // --- Callback 2: Feedback (Updates while running) ---
    send_goal_options.feedback_callback = [this](
      GoalHandleFibonacci::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Next number in sequence received: ";
      for (auto number : feedback->partial_sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    // --- Callback 3: Result (Done) ---
    send_goal_options.result_callback = [this](const GoalHandleFibonacci::WrappedResult & result)
    {
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
      std::stringstream ss;
      ss << "Result received: ";
      for (auto number : result.result->sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};  // class FibonacciActionClient

}  // namespace custom_action_cpp

// --- MAIN FUNCTION TO HANDLE ARGV ---
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Default value if user types nothing
  int user_order = 10; 

  // Check if user provided an argument (e.g., ./client 50)
  if (argc > 1) {
    user_order = atoi(argv[1]);
  } else {
    printf("Usage: ros2 run custom_action_cpp manual_fibonacci_client <order>\n");
    printf("No order specified. Using default: 10\n");
  }

  // Inject the value into the Node via NodeOptions
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"order", user_order}
    }
  );

  auto node = std::make_shared<custom_action_cpp::FibonacciActionClient>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
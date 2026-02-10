#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("whisper_660610842"), count_(0)
  {
    auto param_desc_1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_1.description = "Name of Speaker";
    auto param_desc_2 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_2.description = "Speaking Type";
    auto param_desc_3 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_3.description = "My Word prefix";

    this->declare_parameter("who", "PUB", param_desc_1);
    this->declare_parameter("speaking", "whisper", param_desc_2);
    this->declare_parameter("spk_msg", "Oh My ROS, I am 660610842", param_desc_3);

    publisher_ = this->create_publisher<std_msgs::msg::String>("/gossip_660610842", 10);
    
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();

      std::string my_param_1 = this->get_parameter("who").as_string();
      std::string my_param_2 = this->get_parameter("speaking").as_string();
      
      std::string my_param_3 = this->get_parameter("spk_msg").as_string(); 
          
      this->count_ += 2;

      message.data = my_param_3 +": " +std::to_string(this->count_);

      RCLCPP_INFO(this->get_logger(), "%s %s %s ", 
          my_param_1.c_str(), 
          my_param_2.c_str(), 
          message.data.c_str()); 
      
      this->publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

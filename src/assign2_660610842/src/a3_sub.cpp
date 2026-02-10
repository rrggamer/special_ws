#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LidarInfoSubscriber : public rclcpp::Node
{
public:
  LidarInfoSubscriber()
  : Node("a3_lidar_info")
  {
    auto qos = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos, std::bind(&LidarInfoSubscriber::topic_callback, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "Started Lidar Info Subscriber (Log Throttled 1000ms)...");
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    // 1. Calculate Average (Same logic as before)
    double target_angle = 0.0; 
    double tolerance_deg = 5.0; 
    double tolerance_rad = tolerance_deg * (M_PI / 180.0); 

    double sum_range = 0.0;
    int count = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double current_angle = msg->angle_min + (i * msg->angle_increment);

      if (current_angle >= (target_angle - tolerance_rad) && 
          current_angle <= (target_angle + tolerance_rad)) 
      {
        float r = msg->ranges[i];
        if (!std::isinf(r) && !std::isnan(r) && r >= msg->range_min && r <= msg->range_max) {
          sum_range += r;
          count++;
        }
      }
    }

    double avg_dist = (count > 0) ? (sum_range / count) : 0.0;

    // 2. LOGGING WITH THROTTLE
    // We use *this->get_clock() and set 1000 (milliseconds) to print only once per second.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "\n--- LiDAR Status ---\n"
      "Range Min/Max: %.2f / %.2f m\n"
      "Angle Min/Max: %.2f / %.2f deg\n"
      "Avg Dist (0+/-5deg): %.4f m (pts: %d)\n"
      "--------------------",
      msg->range_min, msg->range_max,
      (msg->angle_min * 180.0 / M_PI), (msg->angle_max * 180.0 / M_PI),
      avg_dist, count
    );
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarInfoSubscriber>());
  rclcpp::shutdown();
  return 0;
}
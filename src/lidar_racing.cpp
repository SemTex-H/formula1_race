#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <string>
#include <vector>

using std::placeholders::_1;

class LidarLeaner : public rclcpp::Node {
public:
  LidarLeaner() : Node("lidar_leaner") {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&LidarLeaner::scan_callback, this, _1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
                "Lidar leaner with 2.0m clearance started!");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  float average_range(const std::vector<float> &ranges, int index,
                      int width = 5) {
    int n = static_cast<int>(ranges.size());
    int start = std::max(index - width, 0);
    int end = std::min(index + width, n - 1);

    std::vector<float> valid;
    for (int i = start; i <= end; ++i) {
      if (ranges[i] > 0.05f && ranges[i] < 20.0f)
        valid.push_back(ranges[i]);
      else
        valid.push_back(21.0f); // replace invalid with max
    }

    if (valid.empty())
      return 20.0f;
    return std::accumulate(valid.begin(), valid.end(), 0.0f) / valid.size();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto &ranges = msg->ranges;
    float left = average_range(ranges, 75);
    float right = average_range(ranges, 25);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.7;
    float speed_a = 0.1;
    float target_distance = 2.0;
    int idx = 0;

    if (left < 10.0) {
      if (left > target_distance) {
        twist.angular.z = speed_a;
        idx = 1;
      } else {
        twist.angular.z = -speed_a;
        idx = 2;
      }
    } else {
      if (right > target_distance) {
        twist.angular.z = -speed_a;
        idx = 3;
      } else {
        twist.angular.z = speed_a;
        idx = 4;
      }
    }

    std::ostringstream log;
    log << "L: " << left << ", R: " << right << ", C: " << idx;
    RCLCPP_INFO(this->get_logger(), log.str().c_str());

    pub_->publish(twist);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLeaner>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PlanningArbitrator : public rclcpp::Node {
public:
  PlanningArbitrator() : Node("planning_arbitrator") {
    local_hold_sec_ = this->declare_parameter<double>("local_hold_sec", 3.0);
    uav_count_ = this->declare_parameter<int>("uav_count", 3);
    global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planner/global_path", 10, std::bind(&PlanningArbitrator::onGlobal, this, std::placeholders::_1));
    local_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planner/local_path", 10, std::bind(&PlanningArbitrator::onLocal, this, std::placeholders::_1));
    for (int i = 1; i <= std::max(1, uav_count_); ++i) {
      cmd_pubs_.push_back(this->create_publisher<nav_msgs::msg::Path>("/uav_" + std::to_string(i) + "/cmd_path", 10));
    }
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/planner/mode", 10);
    tick_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlanningArbitrator::tick, this));
  }

private:
  void onGlobal(const nav_msgs::msg::Path::SharedPtr msg) {
    global_ = *msg;
    has_global_ = true;
  }

  void onLocal(const nav_msgs::msg::Path::SharedPtr msg) {
    local_ = *msg;
    has_local_ = true;
    local_until_ = this->now() + rclcpp::Duration::from_seconds(local_hold_sec_);
  }

  void tick() {
    std_msgs::msg::String mode;
    if (has_local_ && this->now() < local_until_) {
      mode.data = "LOCAL_AVOID";
      local_.header.stamp = this->now();
      for (auto & pub : cmd_pubs_) {
        pub->publish(local_);
      }
      mode_pub_->publish(mode);
      return;
    }

    if (has_global_) {
      mode.data = "GLOBAL_TRACK";
      global_.header.stamp = this->now();
      for (auto & pub : cmd_pubs_) {
        pub->publish(global_);
      }
      mode_pub_->publish(mode);
    }
  }

  double local_hold_sec_;
  int uav_count_;
  bool has_global_ {false};
  bool has_local_ {false};
  rclcpp::Time local_until_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path global_;
  nav_msgs::msg::Path local_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_sub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> cmd_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr tick_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningArbitrator>());
  rclcpp::shutdown();
  return 0;
}

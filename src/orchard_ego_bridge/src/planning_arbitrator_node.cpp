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
    for (int i = 1; i <= std::max(1, uav_count_); ++i) {
      const auto uid = "uav_" + std::to_string(i);
      global_subs_.push_back(this->create_subscription<nav_msgs::msg::Path>(
        "/" + uid + "/planner/global_path",
        10,
        [this, i](const nav_msgs::msg::Path::SharedPtr msg) { this->onGlobal(i - 1, msg); }));
      local_subs_.push_back(this->create_subscription<nav_msgs::msg::Path>(
        "/" + uid + "/planner/local_path",
        10,
        [this, i](const nav_msgs::msg::Path::SharedPtr msg) { this->onLocal(i - 1, msg); }));
      cmd_pubs_.push_back(this->create_publisher<nav_msgs::msg::Path>("/uav_" + std::to_string(i) + "/cmd_path", 10));
      has_global_.push_back(false);
      has_local_.push_back(false);
      global_paths_.emplace_back();
      local_paths_.emplace_back();
      local_until_.emplace_back(0, 0, RCL_ROS_TIME);
    }
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/planner/mode", 10);
    tick_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlanningArbitrator::tick, this));
  }

private:
  void onGlobal(int uav_index, const nav_msgs::msg::Path::SharedPtr msg) {
    if (uav_index < 0 || static_cast<size_t>(uav_index) >= global_paths_.size()) {
      return;
    }
    global_paths_[static_cast<size_t>(uav_index)] = *msg;
    has_global_[static_cast<size_t>(uav_index)] = true;
  }

  void onLocal(int uav_index, const nav_msgs::msg::Path::SharedPtr msg) {
    if (uav_index < 0 || static_cast<size_t>(uav_index) >= local_paths_.size()) {
      return;
    }
    local_paths_[static_cast<size_t>(uav_index)] = *msg;
    has_local_[static_cast<size_t>(uav_index)] = true;
    local_until_[static_cast<size_t>(uav_index)] = this->now() + rclcpp::Duration::from_seconds(local_hold_sec_);
  }

  void tick() {
    bool any_local_active = false;
    const auto stamp = this->now();
    for (size_t i = 0; i < cmd_pubs_.size(); ++i) {
      if (has_local_[i] && stamp < local_until_[i]) {
        any_local_active = true;
        local_paths_[i].header.stamp = stamp;
        cmd_pubs_[i]->publish(local_paths_[i]);
        continue;
      }
      if (has_global_[i]) {
        global_paths_[i].header.stamp = stamp;
        cmd_pubs_[i]->publish(global_paths_[i]);
      }
    }

    std_msgs::msg::String mode;
    mode.data = any_local_active ? "LOCAL_AVOID" : "GLOBAL_TRACK";
    mode_pub_->publish(mode);
  }

  double local_hold_sec_;
  int uav_count_;
  std::vector<bool> has_global_;
  std::vector<bool> has_local_;
  std::vector<rclcpp::Time> local_until_;
  std::vector<nav_msgs::msg::Path> global_paths_;
  std::vector<nav_msgs::msg::Path> local_paths_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> global_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> local_subs_;
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

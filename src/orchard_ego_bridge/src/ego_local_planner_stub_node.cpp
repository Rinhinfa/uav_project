#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class EgoLocalPlannerStub : public rclcpp::Node {
public:
  EgoLocalPlannerStub() : Node("ego_local_planner_stub") {
    trigger_distance_ = this->declare_parameter<double>("trigger_distance", 2.5);
    planner_mode_ = this->declare_parameter<std::string>("planner_mode", "proposed");
    uav_count_ = this->declare_parameter<int>("uav_count", 3);
    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/sim/obstacles", 10, std::bind(&EgoLocalPlannerStub::onObstacles, this, std::placeholders::_1));
    debug_local_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planner/local_path", 10);
    for (int i = 1; i <= std::max(1, uav_count_); ++i) {
      const auto uid = "uav_" + std::to_string(i);
      global_subs_.push_back(this->create_subscription<nav_msgs::msg::Path>(
        "/" + uid + "/planner/global_path",
        10,
        [this, i](const nav_msgs::msg::Path::SharedPtr msg) { this->onGlobalPath(i - 1, msg); }));
      local_pubs_.push_back(this->create_publisher<nav_msgs::msg::Path>(
        "/" + uid + "/planner/local_path", 10));
    }
  }

private:
  void onObstacles(const geometry_msgs::msg::PoseArray::SharedPtr msg) { obstacles_ = *msg; }

  void onGlobalPath(int uav_index, const nav_msgs::msg::Path::SharedPtr msg) {
    if (planner_mode_ == "baseline") {
      return;
    }
    if (msg->poses.empty()) {
      return;
    }
    bool avoid = false;
    nav_msgs::msg::Path local = *msg;
    for (const auto & obs : obstacles_.poses) {
      for (auto & p : local.poses) {
        const double dx = p.pose.position.x - obs.position.x;
        const double dy = p.pose.position.y - obs.position.y;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d < trigger_distance_) {
          avoid = true;
          // 占位策略：横向偏移，接口保持与真实 EgoPlanner 一致。
          p.pose.position.y += (dy >= 0.0 ? 1.2 : -1.2);
        }
      }
    }
    if (avoid) {
      local.header.stamp = this->now();
      if (uav_index >= 0 && static_cast<size_t>(uav_index) < local_pubs_.size()) {
        local_pubs_[static_cast<size_t>(uav_index)]->publish(local);
        debug_local_pub_->publish(local);
      }
    }
  }

  double trigger_distance_;
  int uav_count_;
  std::string planner_mode_;
  geometry_msgs::msg::PoseArray obstacles_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> global_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> local_pubs_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_local_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoLocalPlannerStub>());
  rclcpp::shutdown();
  return 0;
}

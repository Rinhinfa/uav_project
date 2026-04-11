#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class EgoLocalPlannerStub : public rclcpp::Node {
public:
  EgoLocalPlannerStub() : Node("ego_local_planner_stub") {
    trigger_distance_ = this->declare_parameter<double>("trigger_distance", 2.5);
    planner_mode_ = this->declare_parameter<std::string>("planner_mode", "proposed");
    global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planner/global_path", 10, std::bind(&EgoLocalPlannerStub::onGlobalPath, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/sim/obstacles", 10, std::bind(&EgoLocalPlannerStub::onObstacles, this, std::placeholders::_1));
    local_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planner/local_path", 10);
  }

private:
  void onObstacles(const geometry_msgs::msg::PoseArray::SharedPtr msg) { obstacles_ = *msg; }

  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg) {
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
      local_pub_->publish(local);
    }
  }

  double trigger_distance_;
  std::string planner_mode_;
  geometry_msgs::msg::PoseArray obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoLocalPlannerStub>());
  rclcpp::shutdown();
  return 0;
}

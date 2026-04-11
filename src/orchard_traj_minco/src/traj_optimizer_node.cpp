#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using QueryCost = std_srvs::srv::Trigger;

class TrajOptimizerNode : public rclcpp::Node {
public:
  TrajOptimizerNode() : Node("traj_optimizer_minco") {
    keyframe_stride_ = this->declare_parameter<int>("keyframe_stride", 3);
    sample_resolution_ = this->declare_parameter<double>("sample_resolution", 0.25);
    planner_mode_ = this->declare_parameter<std::string>("planner_mode", "proposed");

    coarse_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planner/coarse_waypoints",
      10,
      std::bind(&TrajOptimizerNode::onCoarsePath, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
    cost_srv_ = this->create_service<QueryCost>(
      "/traj/query_cost",
      std::bind(&TrajOptimizerNode::onQueryCost, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onCoarsePath(const nav_msgs::msg::Path::SharedPtr msg) {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      points.push_back(pose.pose.position);
    }
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    if (planner_mode_ == "baseline") {
      path = *msg;
      path.header.stamp = this->now();
    } else {
      path.poses = smoothWithKeyframes(points);
    }
    if (!path.poses.empty()) {
      path_pub_->publish(path);
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> smoothWithKeyframes(
    const std::vector<geometry_msgs::msg::Point> & points) const {
    std::vector<geometry_msgs::msg::Point> keyframes;
    keyframes.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      if (i == 0 || i == points.size() - 1 || static_cast<int>(i % keyframe_stride_) == 0) {
        keyframes.push_back(points[i]);
      }
    }
    std::vector<geometry_msgs::msg::PoseStamped> out;
    if (keyframes.size() < 2) {
      return out;
    }

    for (size_t i = 0; i + 1 < keyframes.size(); ++i) {
      const auto & a = keyframes[i];
      const auto & b = keyframes[i + 1];
      double dx = b.x - a.x;
      double dy = b.y - a.y;
      double dz = b.z - a.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      int steps = std::max(2, static_cast<int>(std::ceil(dist / sample_resolution_)));
      for (int s = 0; s < steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps - 1);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = a.x + t * dx;
        pose.pose.position.y = a.y + t * dy;
        pose.pose.position.z = a.z + t * dz;
        out.push_back(pose);
      }
    }
    return out;
  }

  void onQueryCost(
    const std::shared_ptr<QueryCost::Request> request,
    std::shared_ptr<QueryCost::Response> response) {
    (void)request;
    response->success = true;
    response->message = "Trajectory optimizer alive";
  }

  int keyframe_stride_;
  double sample_resolution_;
  std::string planner_mode_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr coarse_path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<QueryCost>::SharedPtr cost_srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajOptimizerNode>());
  rclcpp::shutdown();
  return 0;
}

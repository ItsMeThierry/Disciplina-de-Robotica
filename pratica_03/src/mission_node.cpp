#include <chrono>
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

struct Waypoint {
  double x;
  double y;
  double theta;
};

class MissionNode : public rclcpp::Node {
public:
  MissionNode() : Node("mission_node"), current_idx_(0), goal_sent_(false), has_odom_(false) {
    std::string goal_topic = declare_parameter("goal_topic", "/goal_pose");
    std::string odom_topic = declare_parameter("odom_topic", "/diff_cont/odom");
    tolerance_xy_ = declare_parameter("tolerance_xy", 0.05);

    // Declarações com fallback padrão limpo
    std::vector<double> waypoints_x = declare_parameter("waypoints_x", std::vector<double>{1.0, 2.0, 0.0});
    std::vector<double> waypoints_y = declare_parameter("waypoints_y", std::vector<double>{1.0, 0.0, 0.0});
    std::vector<double> waypoints_theta = declare_parameter("waypoints_theta", std::vector<double>{0.0, 1.57, 0.0});

    for (size_t i = 0; i < waypoints_x.size(); ++i) {
      waypoints_.push_back({waypoints_x[i], waypoints_y[i], waypoints_theta[i]});
    }

    goal_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(goal_topic, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&MissionNode::odomCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&MissionNode::checkAndSendGoal, this));

    RCLCPP_INFO(get_logger(), "Nó de Missão iniciado com %zu waypoints carregados.", waypoints_.size());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    has_odom_ = true;
  }

  void checkAndSendGoal() {
    if (!has_odom_ || current_idx_ >= waypoints_.size()) return;

    Waypoint target = waypoints_[current_idx_];

    if (!goal_sent_) {
      geometry_msgs::msg::Pose2D goal_msg;
      goal_msg.x = target.x;
      goal_msg.y = target.y;
      goal_msg.theta = target.theta;
      goal_pub_->publish(goal_msg);
      goal_sent_ = true;

      RCLCPP_INFO(get_logger(), "Enviando Waypoint %zu/%zu: (x: %.2f, y: %.2f, theta: %.2f)",
                  current_idx_ + 1, waypoints_.size(), target.x, target.y, target.theta);
      return;
    }

    double dx = target.x - current_x_;
    double dy = target.y - current_y_;
    double dist = std::hypot(dx, dy);

    if (dist <= tolerance_xy_) {
      RCLCPP_INFO(get_logger(), "Waypoint %zu alcançado com sucesso!", current_idx_ + 1);
      current_idx_++;
      goal_sent_ = false;

      if (current_idx_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "--- MISSAO CONCLUIDA ---");
      }
    }
  }

  std::vector<Waypoint> waypoints_;
  size_t current_idx_;
  bool goal_sent_;
  bool has_odom_;
  double current_x_{0.0}, current_y_{0.0};
  double tolerance_xy_;

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
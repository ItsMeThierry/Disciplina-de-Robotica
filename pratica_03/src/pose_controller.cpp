#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class PID {
public:
  PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), first_run_(true) {}

  double compute(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    double p = kp_ * error;
    integral_ += error * dt;
    double i = ki_ * integral_;

    double derivative = 0.0;
    if (!first_run_) {
      derivative = (error - prev_error_) / dt;
    } else {
      first_run_ = false;
    }
    prev_error_ = error;

    return p + i + (kd_ * derivative);
  }

  void reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
    first_run_ = true;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_, integral_;
  bool first_run_;
};

class PoseController : public rclcpp::Node {
public:
  PoseController() : Node("pose_controller") {
    // Parâmetros declarados com valores default de fallback
    kp_linear_ = declare_parameter("kp_linear", 0.5);
    ki_linear_ = declare_parameter("ki_linear", 0.0);
    kd_linear_ = declare_parameter("kd_linear", 0.05);

    kp_angular_ = declare_parameter("kp_angular", 1.5);
    ki_angular_ = declare_parameter("ki_angular", 0.0);
    kd_angular_ = declare_parameter("kd_angular", 0.1);

    max_linear_vel_ = declare_parameter("max_linear_vel", 0.22);
    max_angular_vel_ = declare_parameter("max_angular_vel", 1.0);
    goal_tol_xy_ = declare_parameter("goal_tolerance_xy", 0.05);
    goal_tol_theta_ = declare_parameter("goal_tolerance_theta", 0.1);

    std::string odom_topic = declare_parameter("odom_topic", "/diff_cont/odom");
    std::string cmd_topic = declare_parameter("cmd_vel_topic", "/diff_cont/cmd_vel_unstamped");
    std::string goal_topic = declare_parameter("goal_topic", "/goal_pose");

    pid_lin_ = std::make_unique<PID>(kp_linear_, ki_linear_, kd_linear_);
    pid_ang_ = std::make_unique<PID>(kp_angular_, ki_angular_, kd_angular_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PoseController::odomCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      goal_topic, 10,
      std::bind(&PoseController::goalCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    last_time_ = now();
    timer_ = create_wall_timer(
      50ms, std::bind(&PoseController::controlLoop, this));

    RCLCPP_INFO(get_logger(), "PoseController inicializado.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_theta_);

    has_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    goal_x_ = msg->x;
    goal_y_ = msg->y;
    goal_theta_ = msg->theta;
    has_goal_ = true;
    pid_lin_->reset();
    pid_ang_->reset();
    RCLCPP_INFO(get_logger(),
      "Novo objetivo recebido: x=%.2f, y=%.2f, theta=%.2f", goal_x_, goal_y_, goal_theta_);
  }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  void controlLoop() {
    if (!has_odom_ || !has_goal_) return;

    rclcpp::Time now_t = now();
    double dt = (now_t - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = now_t;

    double dx = goal_x_ - current_x_;
    double dy = goal_y_ - current_y_;
    double dist = std::hypot(dx, dy);

    double angle_to_goal = std::atan2(dy, dx);
    double err_theta_goal = normalizeAngle(angle_to_goal - current_theta_);
    double err_theta_final = normalizeAngle(goal_theta_ - current_theta_);

    geometry_msgs::msg::Twist cmd;

    if (dist > goal_tol_xy_) {
      double w = pid_ang_->compute(err_theta_goal, dt);
      double v = 0.0;
      if (std::fabs(err_theta_goal) < 0.8) {
        v = pid_lin_->compute(dist, dt);
      }
      cmd.linear.x = clamp(v, -max_linear_vel_, max_linear_vel_);
      cmd.angular.z = clamp(w, -max_angular_vel_, max_angular_vel_);
    } else {
      cmd.linear.x = 0.0;
      double w = pid_ang_->compute(err_theta_final, dt);
      cmd.angular.z = clamp(w, -max_angular_vel_, max_angular_vel_);

      if (std::fabs(err_theta_final) < goal_tol_theta_) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        has_goal_ = false;
        RCLCPP_INFO(get_logger(), "Objetivo alcançado!");
      }
    }

    cmd_pub_->publish(cmd);
  }

  double kp_linear_, ki_linear_, kd_linear_;
  double kp_angular_, ki_angular_, kd_angular_;
  double max_linear_vel_, max_angular_vel_;
  double goal_tol_xy_, goal_tol_theta_;

  std::unique_ptr<PID> pid_lin_, pid_ang_;

  double current_x_{0}, current_y_{0}, current_theta_{0};
  double goal_x_{0}, goal_y_{0}, goal_theta_{0};
  bool has_odom_{false}, has_goal_{false};
  rclcpp::Time last_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseController>());
  rclcpp::shutdown();
  return 0;
}
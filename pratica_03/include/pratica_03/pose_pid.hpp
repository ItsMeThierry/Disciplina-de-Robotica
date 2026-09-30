#ifndef POSE_PID_HPP
#define POSE_PID_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "pratica_03/pid.hpp"

namespace pratica
{

class PosePID : public rclcpp::Node
{
public:
    PosePID();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void control_loop();

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) const;
    static double normalize_angle(double angle);

    // Subscribers / Publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Estado
    std::optional<geometry_msgs::msg::Pose> current_pose_;
    std::optional<geometry_msgs::msg::Pose> goal_pose_;
    std::mutex mutex_;

    // PIDs
    std::unique_ptr<PID> pid_distance_;
    std::unique_ptr<PID> pid_yaw_;

    // Controle de tempo
    rclcpp::Time last_time_;
    bool has_last_time_{false};

    // Parâmetros
    double goal_tolerance_xy_;
    double goal_tolerance_yaw_;
    double max_linear_vel_;
    double max_angular_vel_;
};

}

#endif
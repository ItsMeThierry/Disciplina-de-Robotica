#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class PoseControllerNode : public rclcpp::Node
{
public:
    PoseControllerNode() : Node("pose_controller_node")
    {
        // Ganhos vindos do YAML (sem default)
        this->declare_parameter("kp_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kp_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kp_theta", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_theta", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_theta", rclcpp::PARAMETER_DOUBLE);

        double kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_t, ki_t, kd_t;
        try {
            kp_x = this->get_parameter("kp_x").as_double();
            ki_x = this->get_parameter("ki_x").as_double();
            kd_x = this->get_parameter("kd_x").as_double();
            kp_y = this->get_parameter("kp_y").as_double();
            ki_y = this->get_parameter("ki_y").as_double();
            kd_y = this->get_parameter("kd_y").as_double();
            kp_t = this->get_parameter("kp_theta").as_double();
            ki_t = this->get_parameter("ki_theta").as_double();
            kd_t = this->get_parameter("kd_theta").as_double();
        } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
            RCLCPP_FATAL(this->get_logger(),
                "Parâmetro ausente no YAML: %s", e.what());
            throw;
        }

        pid_x_.initPid(kp_x, ki_x, kd_x, 0.5, -0.5, true);
        pid_y_.initPid(kp_y, ki_y, kd_y, 0.5, -0.5, true);
        pid_theta_.initPid(kp_t, ki_t, kd_t, 0.3, -0.3, true);

        RCLCPP_INFO(this->get_logger(), "PID X     -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_x, ki_x, kd_x);
        RCLCPP_INFO(this->get_logger(), "PID Y     -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_y, ki_y, kd_y);
        RCLCPP_INFO(this->get_logger(), "PID Theta -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_t, ki_t, kd_t);

        // Feedback (pose atual) e referência (pose desejada)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robo_vascaino/ground_truth/pose", 10, std::bind(&PoseControllerNode::odom_callback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PoseControllerNode::goal_callback, this, std::placeholders::_1));

        // Saída compatível com diff_drive_controller
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PoseControllerNode::control_loop, this));

        last_time_ = this->now();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = msg->pose;
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Nova pose desejada recebida!");
    }

    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double wrap_to_pi(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    void control_loop()
    {
        if (!has_odom_ || !has_goal_) return;

        // ---------- 1) Erros no frame do mundo ----------
        double ex = target_pose_.position.x - current_pose_.position.x;
        double ey = target_pose_.position.y - current_pose_.position.y;

        double yaw_target  = yaw_from_quaternion(target_pose_.orientation);
        double yaw_current = yaw_from_quaternion(current_pose_.orientation);
        double eth = wrap_to_pi(yaw_target - yaw_current);

        // ---------- 2) Rotacionar o erro para o frame do robô ----------
        // [e_x^b, e_y^b]^T = R(-theta) * [e_x, e_y]^T
        double c = std::cos(yaw_current);
        double s = std::sin(yaw_current);

        double ex_b =  ex * c + ey * s;
        double ey_b = -ex * s + ey * c;
        double eth_b = eth;   // invariante à rotação em Z

        // ---------- dt ----------
        rclcpp::Time now = this->now();
        uint64_t dt_ns = (now - last_time_).nanoseconds();

        last_time_ = now;
        if (dt_ns <= 0) return;

        // ---------- 3) PIDs no frame do robô ----------
        // v_d  = PID(e_x^b)
        // omega_d = PID(e_y^b) + PID(e_theta^b)
        double v_cmd       = pid_x_.computeCommand(ex_b, dt_ns);
        double omega_y     = pid_y_.computeCommand(ey_b, dt_ns);
        double omega_theta = pid_theta_.computeCommand(eth_b, dt_ns);

        double omega_cmd = omega_y + omega_theta;

        // ---------- 4) Publicar comando compatível com diff_drive ----------
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x  = v_cmd;
        twist.linear.y  = 0.0;
        twist.angular.z = omega_cmd;
        cmd_pub_->publish(twist);
    }

    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose target_pose_;
    bool has_odom_ = false;
    bool has_goal_ = false;
    rclcpp::Time last_time_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseControllerNode>());
    rclcpp::shutdown();
    return 0;
}
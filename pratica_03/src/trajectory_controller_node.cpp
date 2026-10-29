#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <stdexcept>

class TrajectoryControllerNode : public rclcpp::Node
{
public:
    TrajectoryControllerNode() : Node("trajectory_controller_node")
    {
        this->declare_parameter("trajectory.A",     rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("trajectory.B",     rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("trajectory.Omega", rclcpp::PARAMETER_DOUBLE);

        try {
            A_     = this->get_parameter("trajectory.A").as_double();
            B_     = this->get_parameter("trajectory.B").as_double();
            Omega_ = this->get_parameter("trajectory.Omega").as_double();
        } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
            RCLCPP_FATAL(this->get_logger(),
                "Parâmetro de trajetória ausente no YAML: %s", e.what());
            throw;
        }

        if (Omega_ <= 0.0) {
            RCLCPP_FATAL(this->get_logger(),
                "trajectory.Omega deve ser > 0 (recebido %.6f)", Omega_);
            throw std::runtime_error("invalid trajectory.Omega");
        }

        // ---------- Modo de operação ----------
        // "open_loop"  -> só feedforward (Kff=1, PIDs zerados)
        // "closed_loop"-> feedforward (Kff) + PIDs
        this->declare_parameter("control.mode", std::string("closed_loop"));
        mode_ = this->get_parameter("control.mode").as_string();
        if (mode_ != "open_loop" && mode_ != "closed_loop") {
            RCLCPP_FATAL(this->get_logger(),
                "control.mode inválido: '%s' (use 'open_loop' ou 'closed_loop')",
                mode_.c_str());
            throw std::runtime_error("invalid control.mode");
        }

        this->declare_parameter("control.Kff", 1.0);
        Kff_ = this->get_parameter("control.Kff").as_double();

        this->declare_parameter("kp_x",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_x",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_x",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kp_y",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_y",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_y",      rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kp_theta",  rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_theta",  rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_theta",  rclcpp::PARAMETER_DOUBLE);

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
            RCLCPP_FATAL(this->get_logger(), "Parâmetro ausente no YAML: %s", e.what());
            throw;
        }

        if (mode_ == "open_loop") {
            kp_x = ki_x = kd_x = 0.0;
            kp_y = ki_y = kd_y = 0.0;
            kp_t = ki_t = kd_t = 0.0;
            Kff_ = 1.0;  // garante feedforward puro
            RCLCPP_WARN(this->get_logger(),
                "Modo OPEN LOOP: PIDs zerados e Kff forçado para 1.0");
        }

        pid_x_.initPid(kp_x, ki_x, kd_x, 0.5, -0.5, true);
        pid_y_.initPid(kp_y, ki_y, kd_y, 0.5, -0.5, true);
        pid_theta_.initPid(kp_t, ki_t, kd_t, 0.3, -0.3, true);

        RCLCPP_INFO(this->get_logger(),
            "Trajetória: A=%.3f B=%.3f Omega=%.3f | mode=%s Kff=%.3f",
            A_, B_, Omega_, mode_.c_str(), Kff_);
        RCLCPP_INFO(this->get_logger(),
            "PID X     -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_x, ki_x, kd_x);
        RCLCPP_INFO(this->get_logger(),
            "PID Y     -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_y, ki_y, kd_y);
        RCLCPP_INFO(this->get_logger(),
            "PID Theta -> Kp=%.3f Ki=%.3f Kd=%.3f", kp_t, ki_t, kd_t);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robo_vascaino/ground_truth/pose", 10,
            std::bind(&TrajectoryControllerNode::odom_callback, this, std::placeholders::_1));

        ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference_pose", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrajectoryControllerNode::control_loop, this));

        start_time_ = this->now();
        last_time_  = this->now();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
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

    void trajectory_reference(double t,
                              double & xd, double & yd, double & thd,
                              double & xd_dot, double & yd_dot,
                              double & xd_ddot, double & yd_ddot)
    {
        const double Ot = Omega_ * t;
        xd      = A_ * std::sin(Ot);
        yd      = B_ * std::sin(2.0 * Ot);
        xd_dot  = A_ * Omega_ * std::cos(Ot);
        yd_dot  = 2.0 * B_ * Omega_ * std::cos(2.0 * Ot);
        xd_ddot = -A_ * Omega_ * Omega_ * std::sin(Ot);
        yd_ddot = -4.0 * B_ * Omega_ * Omega_ * std::sin(2.0 * Ot);
        thd     = std::atan2(yd_dot, xd_dot);
    }

    void control_loop()
    {
        if (!has_odom_) return;

        rclcpp::Time now = this->now();
        uint64_t dt_ns = (now - last_time_).nanoseconds();
        last_time_ = now;
        if (dt_ns <= 0) return;

        const double t = (now - start_time_).seconds();

        // ---------- 1) Referência da trajetória ----------
        double xd, yd, thd, xd_dot, yd_dot, xd_ddot, yd_ddot;
        trajectory_reference(t, xd, yd, thd,
                             xd_dot, yd_dot,
                             xd_ddot, yd_ddot);

        // ---------- 2) Feedforward: velocidade desejada em {B} ----------
        double yaw_curr = yaw_from_quaternion(current_pose_.orientation);
        double c = std::cos(yaw_curr);
        double s = std::sin(yaw_curr);

        double v_ff = std::sqrt(xd_dot * xd_dot + yd_dot * yd_dot);

        const double denom = xd_dot * xd_dot + yd_dot * yd_dot;
        double omega_ff = (xd_dot * yd_ddot - yd_dot * xd_ddot) / denom;

        // ---------- 3) Erros no mundo ----------
        double ex  = xd - current_pose_.position.x;
        double ey  = yd - current_pose_.position.y;
        double eth = wrap_to_pi(thd - yaw_curr);

        // ---------- 4) Erros no frame do robô (iR_r^{-1}) ----------
        double ex_b  =  ex * c + ey * s;
        double ey_b  = -ex * s + ey * c;
        double eth_b = eth;

        // ---------- 5) Realimentação (PIDs) ----------
        double v_fb     = pid_x_.computeCommand(ex_b, dt_ns);
        double omega_y  = pid_y_.computeCommand(ey_b, dt_ns);
        double omega_th = pid_theta_.computeCommand(eth_b, dt_ns);

        // ---------- 6) Feedforward + realimentação ----------
        double v_cmd     = Kff_ * v_ff      + v_fb;
        double omega_cmd = Kff_ * omega_ff  + omega_y + omega_th;

        // ---------- 7) Saturação ----------
        v_cmd     = std::clamp(v_cmd,     0.0, 2.5);
        omega_cmd = std::clamp(omega_cmd, -7.0, 7.0);   

        // ---------- 8) Publica referência ----------
        geometry_msgs::msg::PoseStamped ref_msg;
        ref_msg.header.stamp = now;
        ref_msg.header.frame_id = "odom";
        ref_msg.pose.position.x = xd;
        ref_msg.pose.position.y = yd;
        tf2::Quaternion q_ref;
        q_ref.setRPY(0.0, 0.0, thd);
        ref_msg.pose.orientation.x = q_ref.x();
        ref_msg.pose.orientation.y = q_ref.y();
        ref_msg.pose.orientation.z = q_ref.z();
        ref_msg.pose.orientation.w = q_ref.w();
        ref_pub_->publish(ref_msg);

        // ---------- 9) Publica comando ----------
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x  = v_cmd;
        twist.linear.y  = 0.0;
        twist.angular.z = omega_cmd;
        cmd_pub_->publish(twist);
    }

    double A_, B_, Omega_;
    std::string mode_;
    double Kff_;

    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    geometry_msgs::msg::Pose current_pose_;
    bool has_odom_ = false;
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryControllerNode>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

constexpr double SEMI_EIXO_MAIOR = 6378137.0;
constexpr double ACHATAMENTO = 1.0 / 298.257223563;
constexpr double EXCENTRICIDADE_2 = ACHATAMENTO * (2.0 - ACHATAMENTO);

class PoseControllerNode : public rclcpp::Node
{
public:
    PoseControllerNode() : Node("pose_controller_node")
    {
        this->declare_parameter("kp_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_x", rclcpp::PARAMETER_DOUBLE);

        this->declare_parameter("kp_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_y", rclcpp::PARAMETER_DOUBLE);

        this->declare_parameter("kp_theta", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ki_theta", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kd_theta", rclcpp::PARAMETER_DOUBLE);

        this->declare_parameter("use_feedforward", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("feedforward_gain", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("trajectory_A", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("trajectory_B", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("trajectory_omega", rclcpp::PARAMETER_DOUBLE);

        this->declare_parameter("gps_offset_x", 0.0);
        this->declare_parameter("wheel_separation", 0.26);
        this->declare_parameter("v_wheel_max", 0.8);
        this->declare_parameter("a_lin_max", 0.8);
        this->declare_parameter("a_ang_max", 2.0);

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

            use_feedforward_ = this->get_parameter("use_feedforward").as_bool();
            k_ff_ = this->get_parameter("feedforward_gain").as_double();
            param_A_ = this->get_parameter("trajectory_A").as_double();
            param_B_ = this->get_parameter("trajectory_B").as_double();
            param_omega_ = this->get_parameter("trajectory_omega").as_double();
        } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
            RCLCPP_FATAL(this->get_logger(), "Parâmetro ausente no YAML: %s", e.what());
            throw;
        }

        gps_off_x_ = this->get_parameter("gps_offset_x").as_double();
        wheel_sep_ = this->get_parameter("wheel_separation").as_double();
        v_max_ = this->get_parameter("v_wheel_max").as_double();
        a_lin_ = this->get_parameter("a_lin_max").as_double();
        a_ang_ = this->get_parameter("a_ang_max").as_double();

        pid_x_.initPid(kp_x, ki_x, kd_x, 1.0, -1.0, true);
        pid_y_.initPid(kp_y, ki_y, kd_y, 1.0, -1.0, true);
        pid_theta_.initPid(kp_t, ki_t, kd_t, 1.0, -1.0, true);

        RCLCPP_INFO(this->get_logger(),
                    "Controlador iniciado. Feedforward: %s (k_ff=%.2f, A=%.2f, B=%.2f, w=%.2f)",
                    use_feedforward_ ? "ATIVO" : "INATIVO", k_ff_, param_A_, param_B_, param_omega_);

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&PoseControllerNode::gps_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&PoseControllerNode::imu_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_seconds(0.02),
            [this]() { this->control_loop(); });
    }

private:
    static void geodetica_para_ecef(double lat, double lon, double alt, double &x, double &y, double &z)
    {
        double phi = lat * M_PI / 180.0;
        double lam = lon * M_PI / 180.0;
        double sin_phi = std::sin(phi);
        double n = SEMI_EIXO_MAIOR / std::sqrt(1.0 - EXCENTRICIDADE_2 * sin_phi * sin_phi);

        x = (n + alt) * std::cos(phi) * std::cos(lam);
        y = (n + alt) * std::cos(phi) * std::sin(lam);
        z = (n * (1.0 - EXCENTRICIDADE_2) + alt) * sin_phi;
    }

    void ecef_para_enu(double x, double y, double z, double &east, double &north, double &up) const
    {
        double dx = x - ecef0_x_;
        double dy = y - ecef0_y_;
        double dz = z - ecef0_z_;

        east  = -sin_lam0_ * dx + cos_lam0_ * dy;
        north = -sin_phi0_ * cos_lam0_ * dx - sin_phi0_ * sin_lam0_ * dy + cos_phi0_ * dz;
        up    =  cos_phi0_ * cos_lam0_ * dx + cos_phi0_ * sin_lam0_ * dy + sin_phi0_ * dz;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;

        if (!has_gps_origin_) {
            double lat0 = msg->latitude;
            double lon0 = msg->longitude;
            double alt0 = msg->altitude;

            geodetica_para_ecef(lat0, lon0, alt0, ecef0_x_, ecef0_y_, ecef0_z_);

            double phi0 = lat0 * M_PI / 180.0;
            double lam0 = lon0 * M_PI / 180.0;

            sin_phi0_ = std::sin(phi0);
            cos_phi0_ = std::cos(phi0);
            sin_lam0_ = std::sin(lam0);
            cos_lam0_ = std::cos(lam0);

            has_gps_origin_ = true;
            RCLCPP_INFO(this->get_logger(), "Origem GPS configurada (lat=%.7f, lon=%.7f)", lat0, lon0);
        }

        double x_ecef, y_ecef, z_ecef;
        geodetica_para_ecef(msg->latitude, msg->longitude, msg->altitude, x_ecef, y_ecef, z_ecef);

        double east, north, up;
        ecef_para_enu(x_ecef, y_ecef, z_ecef, east, north, up);

        current_x_ = east;
        current_y_ = north;
        has_gps_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        has_imu_ = true;
    }

    static double wrap_to_pi(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    void limit_cmd(double &v, double &w, double dt)
    {
        const double m = std::abs(v) + std::abs(w) * wheel_sep_ * 0.5;
        if (m > v_max_) {
            const double k = v_max_ / m;
            v *= k;
            w *= k;
        }
        v = std::clamp(v, v_prev_ - a_lin_ * dt, v_prev_ + a_lin_ * dt);
        w = std::clamp(w, w_prev_ - a_ang_ * dt, w_prev_ + a_ang_ * dt);
        v_prev_ = v;
        w_prev_ = w;
    }

    void control_loop()
    {
        if (!has_gps_ || !has_imu_) return;

        const rclcpp::Time now = this->now();

        if (!started_) {
            started_ = true;
            start_time_ = now;
            last_time_ = now;
            return;
        }

        const int64_t dt_ns = (now - last_time_).nanoseconds();
        if (dt_ns <= 0) return;
        last_time_ = now;
        const double dt = static_cast<double>(dt_ns) * 1e-9;
        const uint64_t dt_u = static_cast<uint64_t>(dt_ns);

        const double t = (now - start_time_).seconds();

        const double W = param_omega_;
        const double A = param_A_;
        const double B = param_B_;

        const double xd = A * std::sin(W * t);
        const double yd = B * std::sin(2.0 * W * t);

        const double dxd = A * W * std::cos(W * t);
        const double dyd = 2.0 * B * W * std::cos(2.0 * W * t);
        const double yaw_d = std::atan2(dyd, dxd);

        const double ddxd = -A * W * W * std::sin(W * t);
        const double ddyd = -4.0 * B * W * W * std::sin(2.0 * W * t);

        const double den = dxd * dxd + dyd * dyd;
        const double v_f = std::sqrt(den);
        const double w_f = (den > 1e-6) ? (dxd * ddyd - dyd * ddxd) / den : 0.0;

        const double px = current_x_ + gps_off_x_ * std::cos(current_yaw_);
        const double py = current_y_ + gps_off_x_ * std::sin(current_yaw_);

        const double ex_global = xd - px;
        const double ey_global = yd - py;
        const double eth = wrap_to_pi(yaw_d - current_yaw_);

        const double c = std::cos(current_yaw_);
        const double s = std::sin(current_yaw_);

        const double ex_body =  c * ex_global + s * ey_global;
        const double ey_body = -s * ex_global + c * ey_global;

        const double kff = use_feedforward_ ? k_ff_ : 0.0;

        double v_cmd = kff * v_f * std::cos(eth) + pid_x_.computeCommand(ex_body, dt_u);
        double w_cmd = kff * w_f
                     + v_f * pid_y_.computeCommand(ey_body, dt_u)
                     + pid_theta_.computeCommand(eth, dt_u);

        limit_cmd(v_cmd, w_cmd, dt);

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = v_cmd;
        twist.angular.z = w_cmd;
        cmd_pub_->publish(twist);
    }

    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    double ecef0_x_ = 0.0, ecef0_y_ = 0.0, ecef0_z_ = 0.0;
    double sin_phi0_ = 0.0, cos_phi0_ = 0.0;
    double sin_lam0_ = 0.0, cos_lam0_ = 0.0;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    bool has_gps_origin_ = false;
    bool has_gps_ = false;
    bool has_imu_ = false;
    bool started_ = false;

    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    bool use_feedforward_ = true;
    double k_ff_ = 1.0;
    double param_A_ = 1.0;
    double param_B_ = 1.0;
    double param_omega_ = 0.2;

    double gps_off_x_ = 0.0;
    double wheel_sep_ = 0.26;
    double v_max_ = 0.8;
    double a_lin_ = 0.8;
    double a_ang_ = 2.0;
    double v_prev_ = 0.0;
    double w_prev_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
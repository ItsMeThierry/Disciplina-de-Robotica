#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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
            Kff_ = 1.0;
            RCLCPP_WARN(this->get_logger(),
                "Modo OPEN LOOP: PIDs zerados e Kff forçado para 1.0");
        }

        pid_x_.initPid(kp_x, ki_x, kd_x, 0.5, -0.5, true);
        pid_y_.initPid(kp_y, ki_y, kd_y, 0.5, -0.5, true);
        pid_theta_.initPid(kp_t, ki_t, kd_t, 0.3, -0.3, true);

        RCLCPP_INFO(this->get_logger(),
            "Trajetória: A=%.3f B=%.3f Omega=%.3f | mode=%s Kff=%.3f",
            A_, B_, Omega_, mode_.c_str(), Kff_);

        // Subscrições dos sensores
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&TrajectoryControllerNode::gps_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&TrajectoryControllerNode::imu_callback, this, std::placeholders::_1));

        ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference_pose", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrajectoryControllerNode::control_loop, this));

        start_time_ = this->now();
        last_time_  = this->now();
    }

private:
    // ---------- Funções de Geodésia (WGS84 para ECEF e ENU) ----------
    void geodetica_para_ecef(double lat, double lon, double alt, double &x, double &y, double &z)
    {
        double phi = lat * (M_PI / 180.0);
        double lam = lon * (M_PI / 180.0);
        double sin_phi = std::sin(phi);
        
        double n = SEMI_EIXO_MAIOR / std::sqrt(1.0 - EXCENTRICIDADE_2 * sin_phi * sin_phi);
        x = (n + alt) * std::cos(phi) * std::cos(lam);
        y = (n + alt) * std::cos(phi) * std::sin(lam);
        z = (n * (1.0 - EXCENTRICIDADE_2) + alt) * sin_phi;
    }

    void inicializar_origem_gps(double lat, double lon, double alt)
    {
        lat0_ = lat;
        lon0_ = lon;
        alt0_ = alt;

        geodetica_para_ecef(lat0_, lon0_, alt0_, x0_ecef_, y0_ecef_, z0_ecef_);

        double phi = lat0_ * (M_PI / 180.0);
        double lam = lon0_ * (M_PI / 180.0);
        sin_phi0_ = std::sin(phi);
        cos_phi0_ = std::cos(phi);
        sin_lam0_ = std::sin(lam);
        cos_lam0_ = std::cos(lam);

        has_gps_origin_ = true;
        RCLCPP_INFO(this->get_logger(), "Origem GPS (Datum) inicializada em: lat=%.7f, lon=%.7f", lat0_, lon0_);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            return;
        }

        // Define a primeira leitura válida como a origem local (0,0,0)
        if (!has_gps_origin_) {
            inicializar_origem_gps(msg->latitude, msg->longitude, msg->altitude);
        }

        // Converte a posição atual para ECEF
        double x_ecef, y_ecef, z_ecef;
        geodetica_para_ecef(msg->latitude, msg->longitude, msg->altitude, x_ecef, y_ecef, z_ecef);

        // Diferença em relação à origem
        double dx = x_ecef - x0_ecef_;
        double dy = y_ecef - y0_ecef_;
        double dz = z_ecef - z0_ecef_;

        // Projeção ENU (East-North-Up)
        double leste  = -sin_lam0_ * dx + cos_lam0_ * dy;
        double norte  = -sin_phi0_ * cos_lam0_ * dx - sin_phi0_ * sin_lam0_ * dy + cos_phi0_ * dz;

        // Atribui ao referencial cartesiano do robô (X = Leste, Y = Norte)
        current_x_ = leste;
        current_y_ = norte;
        has_gps_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_orientation_ = msg->orientation;
        has_imu_ = true;
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
        if (!has_gps_ || !has_imu_) return;

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

        // ---------- 2) Orientação Atual (IMU) ----------
        double yaw_curr = yaw_from_quaternion(current_orientation_);
        double c = std::cos(yaw_curr);
        double s = std::sin(yaw_curr);

        double v_ff = std::sqrt(xd_dot * xd_dot + yd_dot * yd_dot);

        const double denom = xd_dot * xd_dot + yd_dot * yd_dot;
        double omega_ff = (xd_dot * yd_ddot - yd_dot * xd_ddot) / denom;

        // ---------- 3) Erros no mundo ----------
        double ex  = xd - current_x_;
        double ey  = yd - current_y_;
        double eth = wrap_to_pi(thd - yaw_curr);

        // ---------- 4) Erros no frame do robô ----------
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
        ref_msg.header.frame_id = "world";
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

    // Variáveis de estado do GPS e Elipsoide WGS84
    bool has_gps_origin_ = false;
    double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
    double x0_ecef_ = 0.0, y0_ecef_ = 0.0, z0_ecef_ = 0.0;
    double sin_phi0_ = 0.0, cos_phi0_ = 0.0, sin_lam0_ = 0.0, cos_lam0_ = 0.0;

    const double SEMI_EIXO_MAIOR = 6378137.0;
    const double ACHATAMENTO = 1.0 / 298.257223563;
    const double EXCENTRICIDADE_2 = (1.0 / 298.257223563) * (2.0 - (1.0 / 298.257223563));

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    geometry_msgs::msg::Quaternion current_orientation_;
    
    bool has_gps_ = false;
    bool has_imu_ = false;

    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
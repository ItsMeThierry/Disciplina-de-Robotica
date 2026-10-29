#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <stdexcept>

class TrajectoryControllerNode : public rclcpp::Node
{
public:
    TrajectoryControllerNode() : Node("trajectory_controller_node")
    {
        // ---------- Trajetória ----------
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

        // ---------- Sensores ----------
        this->declare_parameter("sensors.gps_topic", std::string("/gps/fix"));
        this->declare_parameter("sensors.imu_topic", std::string("/imu/data"));
        // Se qualquer sensor ficar mais que este tempo (s) sem dados, o robô para.
        this->declare_parameter("sensors.timeout",   0.5);

        // Origem do ENU:
        //  "first_fix": o 1º fix válido vira a origem; assume-se que nesse instante
        //               o robô está em (gps.initial_x, gps.initial_y) no frame do mundo
        //               (use isso se o robô nasce em (0,0) no Gazebo).
        //  "fixed"    : usa gps.origin_lat/lon/alt. Devem ser iguais ao
        //               <spherical_coordinates> do .world (origem do mundo).
        this->declare_parameter("gps.origin_mode", std::string("first_fix"));
        this->declare_parameter("gps.origin_lat",  0.0);
        this->declare_parameter("gps.origin_lon",  0.0);
        this->declare_parameter("gps.origin_alt",  0.0);
        // Ângulo (graus, anti-horário) do eixo +x do MUNDO medido a partir do Leste.
        //   0   -> x = Leste, y = Norte
        //   90  -> x = Norte, y = Oeste
        //   180 -> x = Oeste, y = Sul
        // Corresponde ao heading_deg de <spherical_coordinates> do .world (ver README/resposta).
        this->declare_parameter("gps.world_heading_deg", 0.0);
        this->declare_parameter("gps.initial_x",   0.0);
        this->declare_parameter("gps.initial_y",   0.0);
        // Posição da antena GPS em {B} em relação ao ponto de controle (centro do eixo).
        // No URDF atual o gps_link está em (0, 0, 0.06) de base_link -> offset 0.
        this->declare_parameter("gps.lever_arm_x", 0.0);
        this->declare_parameter("gps.lever_arm_y", 0.0);
        // Correção de yaw da IMU (rad), caso o frame de referência da IMU esteja girado.
        this->declare_parameter("imu.yaw_offset",  0.0);
        // Compara com o ground truth só para log/validação (não entra no controle).
        this->declare_parameter("debug.compare_ground_truth", true);

        const std::string gps_topic = this->get_parameter("sensors.gps_topic").as_string();
        const std::string imu_topic = this->get_parameter("sensors.imu_topic").as_string();
        sensor_timeout_ = this->get_parameter("sensors.timeout").as_double();

        origin_mode_ = this->get_parameter("gps.origin_mode").as_string();
        if (origin_mode_ != "first_fix" && origin_mode_ != "fixed") {
            RCLCPP_FATAL(this->get_logger(),
                "gps.origin_mode inválido: '%s' (use 'first_fix' ou 'fixed')",
                origin_mode_.c_str());
            throw std::runtime_error("invalid gps.origin_mode");
        }
        if (origin_mode_ == "fixed") {
            set_origin(this->get_parameter("gps.origin_lat").as_double(),
                       this->get_parameter("gps.origin_lon").as_double(),
                       this->get_parameter("gps.origin_alt").as_double());
        }
        {
            const double th = this->get_parameter("gps.world_heading_deg").as_double()
                              * M_PI / 180.0;
            cos_heading_ = std::cos(th);
            sin_heading_ = std::sin(th);
        }
        initial_x_   = this->get_parameter("gps.initial_x").as_double();
        initial_y_   = this->get_parameter("gps.initial_y").as_double();
        lever_x_     = this->get_parameter("gps.lever_arm_x").as_double();
        lever_y_     = this->get_parameter("gps.lever_arm_y").as_double();
        yaw_offset_  = this->get_parameter("imu.yaw_offset").as_double();
        compare_gt_  = this->get_parameter("debug.compare_ground_truth").as_bool();

        // ---------- Ganhos ----------
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
        RCLCPP_INFO(this->get_logger(),
            "Sensores: GPS='%s' IMU='%s' origem=%s timeout=%.2fs",
            gps_topic.c_str(), imu_topic.c_str(), origin_mode_.c_str(), sensor_timeout_);

        // ---------- Comunicação ----------
        // SensorDataQoS (best effort) é compatível com publishers reliable e best effort.
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, rclcpp::SensorDataQoS(),
            std::bind(&TrajectoryControllerNode::gps_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&TrajectoryControllerNode::imu_callback, this, std::placeholders::_1));

        if (compare_gt_) {
            gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/robo_vascaino/ground_truth/pose", 10,
                std::bind(&TrajectoryControllerNode::gt_callback, this, std::placeholders::_1));
        }

        ref_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference_pose", 10);
        meas_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sensor_pose", 10);
        cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrajectoryControllerNode::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();  // reiniciado quando os sensores ficarem prontos
    }

private:
    // =====================================================================
    //  Conversões
    // =====================================================================
    // WGS-84
    static constexpr double WGS84_A  = 6378137.0;
    static constexpr double WGS84_F  = 1.0 / 298.257223563;

    static void geodetic_to_ecef(double lat_deg, double lon_deg, double h,
                                 double & X, double & Y, double & Z)
    {
        const double e2  = WGS84_F * (2.0 - WGS84_F);
        const double lat = lat_deg * M_PI / 180.0;
        const double lon = lon_deg * M_PI / 180.0;
        const double sl  = std::sin(lat);
        const double N   = WGS84_A / std::sqrt(1.0 - e2 * sl * sl);
        X = (N + h) * std::cos(lat) * std::cos(lon);
        Y = (N + h) * std::cos(lat) * std::sin(lon);
        Z = (N * (1.0 - e2) + h) * sl;
    }

    void set_origin(double lat_deg, double lon_deg, double alt)
    {
        origin_lat_ = lat_deg * M_PI / 180.0;
        origin_lon_ = lon_deg * M_PI / 180.0;
        geodetic_to_ecef(lat_deg, lon_deg, alt, X0_, Y0_, Z0_);
        has_origin_ = true;
    }

    // lat/lon/alt -> ENU (e = Leste, n = Norte) relativo à origem
    void gps_to_enu(double lat_deg, double lon_deg, double alt,
                    double & e, double & n) const
    {
        double X, Y, Z;
        geodetic_to_ecef(lat_deg, lon_deg, alt, X, Y, Z);
        const double dx = X - X0_, dy = Y - Y0_, dz = Z - Z0_;
        const double sp = std::sin(origin_lat_), cp = std::cos(origin_lat_);
        const double sl = std::sin(origin_lon_), cl = std::cos(origin_lon_);
        e = -sl * dx + cl * dy;
        n = -sp * cl * dx - sp * sl * dy + cp * dz;
    }

    // yaw (rotação em torno de z) direto do quaternion, robusto a roll/pitch
    static double yaw_from_quaternion(double x, double y, double z, double w)
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    static double wrap_to_pi(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    // =====================================================================
    //  Callbacks dos sensores
    // =====================================================================
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) return;
        const double alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;

        if (!has_origin_) {  // modo first_fix
            set_origin(msg->latitude, msg->longitude, alt);
            RCLCPP_INFO(this->get_logger(),
                "Origem GPS fixada no 1º fix: lat=%.8f lon=%.8f alt=%.2f "
                "(robô assumido em x=%.2f y=%.2f)",
                msg->latitude, msg->longitude, alt, initial_x_, initial_y_);
        }

        double e, n;
        gps_to_enu(msg->latitude, msg->longitude, alt, e, n);

        // ENU -> eixos do mundo (rotação por world_heading_deg)
        gps_x_ = initial_x_ + ( cos_heading_ * e + sin_heading_ * n);
        gps_y_ = initial_y_ + (-sin_heading_ * e + cos_heading_ * n);
        gps_stamp_ = this->now();
        has_gps_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto & q = msg->orientation;
        const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!(n2 > 0.5) || !std::isfinite(n2)) return;  // orientação inválida/não preenchida

        const double inv = 1.0 / std::sqrt(n2);
        imu_yaw_ = wrap_to_pi(
            yaw_from_quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv) + yaw_offset_);
        imu_stamp_ = this->now();
        has_imu_ = true;
    }

    // Apenas para validação (log). Não é usado no controle.
    void gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        gt_x_ = msg->pose.pose.position.x;
        gt_y_ = msg->pose.pose.position.y;
        const auto & q = msg->pose.pose.orientation;
        gt_yaw_ = yaw_from_quaternion(q.x, q.y, q.z, q.w);
        has_gt_ = true;
    }

    void publish_zero_cmd()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
    }

    // =====================================================================
    //  Trajetória
    // =====================================================================
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

    // =====================================================================
    //  Malha de controle
    // =====================================================================
    void control_loop()
    {
        if (!has_gps_ || !has_imu_) return;

        rclcpp::Time now = this->now();

        // Primeira vez com sensores prontos: a trajetória começa agora (t = 0)
        if (!sensors_ready_) {
            sensors_ready_ = true;
            start_time_ = now;
            last_time_  = now;
            RCLCPP_INFO(this->get_logger(), "GPS e IMU prontos. Iniciando trajetória.");
            return;
        }

        // Watchdog dos sensores
        if ((now - gps_stamp_).seconds() > sensor_timeout_ ||
            (now - imu_stamp_).seconds() > sensor_timeout_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Sensor (GPS/IMU) sem dados há mais de %.2fs: parando o robô.",
                sensor_timeout_);
            publish_zero_cmd();
            last_time_ = now;  // evita dt gigante quando os dados voltarem
            return;
        }

        const int64_t dt_ns = (now - last_time_).nanoseconds();
        last_time_ = now;
        if (dt_ns <= 0) return;

        const double t = (now - start_time_).seconds();

        // ---------- 0) Estado medido pelos sensores ----------
        const double yaw_curr = imu_yaw_;
        const double c = std::cos(yaw_curr);
        const double s = std::sin(yaw_curr);

        // Antena -> ponto de controle: p_robo = p_gps - R(yaw) * lever_arm
        const double x_curr = gps_x_ - (c * lever_x_ - s * lever_y_);
        const double y_curr = gps_y_ - (s * lever_x_ + c * lever_y_);

        // ---------- 1) Referência da trajetória ----------
        double xd, yd, thd, xd_dot, yd_dot, xd_ddot, yd_ddot;
        trajectory_reference(t, xd, yd, thd,
                             xd_dot, yd_dot,
                             xd_ddot, yd_ddot);

        // ---------- 2) Feedforward: velocidade desejada em {B} ----------
        const double denom = xd_dot * xd_dot + yd_dot * yd_dot;
        double v_ff     = std::sqrt(denom);
        double omega_ff = (xd_dot * yd_ddot - yd_dot * xd_ddot) / denom;

        // ---------- 3) Erros no mundo ----------
        double ex  = xd - x_curr;
        double ey  = yd - y_curr;
        double eth = wrap_to_pi(thd - yaw_curr);

        // ---------- 4) Erros no frame do robô (iR_r^{-1}) ----------
        double ex_b  =  ex * c + ey * s;
        double ey_b  = -ex * s + ey * c;
        double eth_b = eth;

        // ---------- 5) Realimentação (PIDs) ----------
        double v_fb     = pid_x_.computeCommand(ex_b, static_cast<uint64_t>(dt_ns));
        double omega_y  = pid_y_.computeCommand(ey_b, static_cast<uint64_t>(dt_ns));
        double omega_th = pid_theta_.computeCommand(eth_b, static_cast<uint64_t>(dt_ns));

        // ---------- 6) Feedforward + realimentação ----------
        double v_cmd     = Kff_ * v_ff      + v_fb;
        double omega_cmd = Kff_ * omega_ff  + omega_y + omega_th;

        // ---------- 7) Saturação ----------
        v_cmd     = std::clamp(v_cmd,     0.0, 2.5);
        omega_cmd = std::clamp(omega_cmd, -7.0, 7.0);

        // ---------- 8) Publica referência e pose medida ----------
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

        geometry_msgs::msg::PoseStamped meas_msg;
        meas_msg.header.stamp = now;
        meas_msg.header.frame_id = "odom";
        meas_msg.pose.position.x = x_curr;
        meas_msg.pose.position.y = y_curr;
        tf2::Quaternion q_meas;
        q_meas.setRPY(0.0, 0.0, yaw_curr);
        meas_msg.pose.orientation.x = q_meas.x();
        meas_msg.pose.orientation.y = q_meas.y();
        meas_msg.pose.orientation.z = q_meas.z();
        meas_msg.pose.orientation.w = q_meas.w();
        meas_pub_->publish(meas_msg);

        // ---------- 9) Publica comando ----------
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x  = v_cmd;
        twist.linear.y  = 0.0;
        twist.angular.z = omega_cmd;
        cmd_pub_->publish(twist);

        // ---------- 10) Validação contra ground truth (só log) ----------
        if (compare_gt_ && has_gt_) {
            if (std::hypot(x_curr - gt_x_, y_curr - gt_y_) > 1.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "GPS diverge do ground truth em %.2f m. Verifique gps.world_heading_deg, "
                    "gps.initial_x/y e se o robô nasceu em (0,0).",
                    std::hypot(x_curr - gt_x_, y_curr - gt_y_));
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Sensor vs GT -> dx=%.3f dy=%.3f dyaw=%.3f rad | erro de rastreio: ex=%.3f ey=%.3f eth=%.3f",
                x_curr - gt_x_, y_curr - gt_y_, wrap_to_pi(yaw_curr - gt_yaw_),
                ex, ey, eth);
        }
    }

    // ---------- Parâmetros / estado ----------
    double A_, B_, Omega_;
    std::string mode_;
    double Kff_;

    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    // GPS
    std::string origin_mode_;
    bool   has_origin_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0;
    double X0_ = 0.0, Y0_ = 0.0, Z0_ = 0.0;
    double cos_heading_ = 1.0, sin_heading_ = 0.0;
    double initial_x_ = 0.0, initial_y_ = 0.0;
    double lever_x_ = 0.0, lever_y_ = 0.0;
    double gps_x_ = 0.0, gps_y_ = 0.0;
    bool   has_gps_ = false;
    rclcpp::Time gps_stamp_{0, 0, RCL_ROS_TIME};

    // IMU
    double yaw_offset_ = 0.0;
    double imu_yaw_ = 0.0;
    bool   has_imu_ = false;
    rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};

    // Ground truth (somente debug)
    bool   compare_gt_ = true;
    bool   has_gt_ = false;
    double gt_x_ = 0.0, gt_y_ = 0.0, gt_yaw_ = 0.0;

    double sensor_timeout_ = 0.5;
    bool   sensors_ready_ = false;

    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr meas_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryControllerNode>());
    rclcpp::shutdown();
    return 0;
}
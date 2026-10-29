#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>

class MissionFollowerNode : public rclcpp::Node
{
public:
    MissionFollowerNode() : Node("mission_follower_node")
    {
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<double>("position_tolerance", std::nan(""));
        this->declare_parameter<double>("yaw_tolerance",      std::nan(""));
        this->declare_parameter<double>("waypoint_timeout",   std::nan(""));
        this->declare_parameter<double>("dwell_time",         std::nan(""));
        this->declare_parameter<bool>("loop",                 false);

        this->get_parameter("mission_file", mission_file_);

        if (mission_file_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "Parâmetro 'mission_file' não definido! Defina o caminho do YAML.");
            throw std::runtime_error("mission_file vazio");
        }

        // Carrega YAML (sobrescreve tudo que vier do YAML; valida no fim)
        if (!load_mission(mission_file_)) {
            RCLCPP_FATAL(this->get_logger(),
                "Falha ao carregar missão do arquivo: %s", mission_file_.c_str());
            throw std::runtime_error("missão inválida");
        }

        // Validação final: se algo ainda é NaN/ inválido, aborta
        if (!std::isfinite(pos_tol_) || pos_tol_ <= 0.0) {
            RCLCPP_FATAL(this->get_logger(),
                "position_tolerance inválido (%.3f). Defina no YAML.", pos_tol_);
            throw std::runtime_error("position_tolerance inválido");
        }
        if (!std::isfinite(yaw_tol_) || yaw_tol_ <= 0.0) {
            RCLCPP_FATAL(this->get_logger(),
                "yaw_tolerance inválido (%.3f). Defina no YAML.", yaw_tol_);
            throw std::runtime_error("yaw_tolerance inválido");
        }
        if (!std::isfinite(wp_timeout_) || wp_timeout_ < 0.0) {
            RCLCPP_FATAL(this->get_logger(),
                "waypoint_timeout inválido (%.3f). Defina no YAML.", wp_timeout_);
            throw std::runtime_error("waypoint_timeout inválido");
        }
        if (!std::isfinite(dwell_time_) || dwell_time_ < 0.0) {
            RCLCPP_FATAL(this->get_logger(),
                "dwell_time inválido (%.3f). Defina no YAML.", dwell_time_);
            throw std::runtime_error("dwell_time inválido");
        }
        if (frame_id_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "frame_id vazio. Defina 'mission.frame_id' no YAML.");
            throw std::runtime_error("frame_id vazio");
        }

        RCLCPP_INFO(this->get_logger(),
            "Missão carregada: %zu waypoints (loop=%s), frame='%s'",
            waypoints_.size(), loop_ ? "true" : "false", frame_id_.c_str());

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robo_vascaino/ground_truth/pose", 10,
            std::bind(&MissionFollowerNode::odom_callback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionFollowerNode::state_machine, this));

        state_ = State::IDLE;
        last_state_change_ = this->now();
    }

private:
    enum class State { IDLE, SEND_GOAL, MOVING, DWELL, DONE };

    struct Waypoint {
        std::string name;
        geometry_msgs::msg::Pose pose;
    };

    // ---------------- Carregar YAML ----------------
    bool load_mission(const std::string & path)
    {
        YAML::Node root;
        try {
            root = YAML::LoadFile(path);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Erro lendo YAML: %s", e.what());
            return false;
        }

        if (!root["mission"]) {
            RCLCPP_ERROR(this->get_logger(), "Arquivo YAML sem chave 'mission'");
            return false;
        }

        auto mission = root["mission"];

        auto read_double = [&](const char * key, double & out) {
            if (root[key]) { out = root[key].as<double>(); return true; }
            if (mission[key]) { out = mission[key].as<double>(); return true; }
            return false;
        };
        auto read_bool = [&](const char * key, bool & out) {
            if (root[key]) { out = root[key].as<bool>(); return true; }
            if (mission[key]) { out = mission[key].as<bool>(); return true; }
            return false;
        };

        read_double("position_tolerance", pos_tol_);
        read_double("yaw_tolerance",      yaw_tol_);
        read_double("waypoint_timeout",   wp_timeout_);
        read_double("dwell_time",         dwell_time_);
        read_bool  ("loop",               loop_);

        if (root["frame_id"])          frame_id_ = root["frame_id"].as<std::string>();
        else if (mission["frame_id"])  frame_id_ = mission["frame_id"].as<std::string>();
        else {
            RCLCPP_WARN(this->get_logger(),
                "frame_id não definido no YAML; usando default '%s'. "
                "Recomendado definir no YAML.", frame_id_.c_str());
        }

        auto wps = mission["waypoints"];
        if (!wps || !wps.IsSequence() || wps.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Sem waypoints na missão");
            return false;
        }

        for (size_t i = 0; i < wps.size(); ++i) {
            const auto & w = wps[i];
            Waypoint wp;
            wp.name = w["name"] ? w["name"].as<std::string>()
                                : ("wp_" + std::to_string(i));

            auto pos = w["position"];
            if (!pos) {
                RCLCPP_ERROR(this->get_logger(),
                    "Waypoint '%s' sem 'position'", wp.name.c_str());
                return false;
            }
            wp.pose.position.x = pos["x"] ? pos["x"].as<double>() : 0.0;
            wp.pose.position.y = pos["y"] ? pos["y"].as<double>() : 0.0;
            wp.pose.position.z = pos["z"] ? pos["z"].as<double>() : 0.0;
            auto ori = w["orientation"];
            double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
            if (ori) {
                if (ori["x"]) qx = ori["x"].as<double>();
                if (ori["y"]) qy = ori["y"].as<double>();
                if (ori["z"]) qz = ori["z"].as<double>();
                if (ori["w"]) qw = ori["w"].as<double>();
            }

            // Normaliza / valida: se norma ~0, força identidade
            double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            if (norm < 1e-6 || !std::isfinite(norm)) {
                RCLCPP_WARN(this->get_logger(),
                    "Waypoint '%s': quaternion inválido (norma=%.3e). "
                    "Usando identidade (0,0,0,1).", wp.name.c_str(), norm);
                qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0;
            } else {
                qx /= norm; qy /= norm; qz /= norm; qw /= norm;
            }

            wp.pose.orientation.x = qx;
            wp.pose.orientation.y = qy;
            wp.pose.orientation.z = qz;
            wp.pose.orientation.w = qw;

            waypoints_.push_back(wp);
        }

        RCLCPP_INFO(this->get_logger(),
            "Tolerâncias (YAML): pos=%.3f m, yaw=%.3f rad, timeout=%.1f s, dwell=%.1f s",
            pos_tol_, yaw_tol_, wp_timeout_, dwell_time_);
        return true;
    }

    // ---------------- Callbacks ----------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    // ---------------- Utilidades ----------------
    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
    {
        double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (norm < 1e-6 || !std::isfinite(norm)) {
            return 0.0;
        }

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        quat.normalize();
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (!std::isfinite(yaw)) {
            return 0.0;
        }
        return yaw;
    }

    static double wrap_to_pi(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    bool reached_waypoint(const geometry_msgs::msg::Pose & target,
                          double & dist_err, double & yaw_err) const
    {
        double dx = target.position.x - current_pose_.position.x;
        double dy = target.position.y - current_pose_.position.y;
        dist_err = std::hypot(dx, dy);

        double yaw_t = yaw_from_quaternion(target.orientation);
        double yaw_c = yaw_from_quaternion(current_pose_.orientation);
        yaw_err = std::fabs(wrap_to_pi(yaw_t - yaw_c));

        if (!std::isfinite(dist_err) || !std::isfinite(yaw_err)) {
            return false;  // proteção extra
        }

        return (dist_err <= pos_tol_) && (yaw_err <= yaw_tol_);
    }

    void publish_goal(const Waypoint & wp)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        msg.pose = wp.pose;

        // sanity do quaternion do goal antes de publicar
        double n = std::sqrt(
            wp.pose.orientation.x*wp.pose.orientation.x +
            wp.pose.orientation.y*wp.pose.orientation.y +
            wp.pose.orientation.z*wp.pose.orientation.z +
            wp.pose.orientation.w*wp.pose.orientation.w);
        if (n < 1e-6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Goal com quaternion inválido; forçando identidade.");
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
            msg.pose.orientation.w = 1.0;
        }

        goal_pub_->publish(msg);
    }

    double elapsed_since(rclcpp::Time t) const
    {
        return (this->now() - t).seconds();
    }

    // ---------------- Máquina de estados ----------------
    void state_machine()
    {
        if (!has_odom_) return;

        switch (state_) {
            case State::IDLE:
                if (waypoints_.empty()) {
                    state_ = State::DONE;
                } else {
                    current_index_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                        "Iniciando missão. Waypoint 1/%zu: %s",
                        waypoints_.size(), waypoints_[current_index_].name.c_str());
                    state_ = State::SEND_GOAL;
                    last_state_change_ = this->now();
                }
                break;

            case State::SEND_GOAL:
                publish_goal(waypoints_[current_index_]);
                last_state_change_ = this->now();
                state_ = State::MOVING;
                break;

            case State::MOVING: {
                double dist_err = 0.0, yaw_err = 0.0;
                if (reached_waypoint(waypoints_[current_index_].pose, dist_err, yaw_err)) {
                    RCLCPP_INFO(this->get_logger(),
                        "Waypoint %zu (%s) alcançado (dist=%.3f m, yaw=%.3f rad)",
                        current_index_ + 1, waypoints_[current_index_].name.c_str(),
                        dist_err, yaw_err);
                    last_state_change_ = this->now();
                    state_ = State::DWELL;
                    break;
                }

                if (wp_timeout_ > 0.0 &&
                    elapsed_since(last_state_change_) > wp_timeout_) {
                    RCLCPP_WARN(this->get_logger(),
                        "Timeout no waypoint %zu (%s) após %.1f s "
                        "(dist=%.3f m, yaw=%.3f rad). Avançando.",
                        current_index_ + 1, waypoints_[current_index_].name.c_str(),
                        elapsed_since(last_state_change_), dist_err, yaw_err);
                    last_state_change_ = this->now();
                    state_ = State::DWELL;
                }
                break;
            }

            case State::DWELL:
                if (elapsed_since(last_state_change_) >= dwell_time_) {
                    current_index_++;
                    if (current_index_ >= waypoints_.size()) {
                        if (loop_) {
                            current_index_ = 0;
                            RCLCPP_INFO(this->get_logger(),
                                "Missão concluída. Reiniciando (loop).");
                            state_ = State::SEND_GOAL;
                            last_state_change_ = this->now();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Missão concluída!");
                            state_ = State::DONE;
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                            "Próximo waypoint %zu/%zu: %s",
                            current_index_ + 1, waypoints_.size(),
                            waypoints_[current_index_].name.c_str());
                        state_ = State::SEND_GOAL;
                        last_state_change_ = this->now();
                    }
                }
                break;

            case State::DONE:
                break;
        }
    }

    // ---------------- Membros ----------------
    std::string mission_file_;
    std::string frame_id_ = "odom";
    double pos_tol_ = std::nan("");
    double yaw_tol_ = std::nan("");
    double wp_timeout_ = std::nan("");
    double dwell_time_ = std::nan("");
    bool loop_ = false;

    std::vector<Waypoint> waypoints_;
    size_t current_index_ = 0;
    State state_ = State::IDLE;
    rclcpp::Time last_state_change_;

    geometry_msgs::msg::Pose current_pose_;
    bool has_odom_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
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

class MissionFollowerNode : public rclcpp::Node
{
public:
    MissionFollowerNode() : Node("mission_follower_node")
    {
        // Parâmetros
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<double>("position_tolerance", 0.10);
        this->declare_parameter<double>("yaw_tolerance", 0.15);
        this->declare_parameter<double>("waypoint_timeout", 60.0);
        this->declare_parameter<double>("dwell_time", 0.5);
        this->declare_parameter<bool>("loop", false);

        this->get_parameter("mission_file", mission_file_);
        this->get_parameter("position_tolerance", pos_tol_);
        this->get_parameter("yaw_tolerance", yaw_tol_);
        this->get_parameter("waypoint_timeout", wp_timeout_);
        this->get_parameter("dwell_time", dwell_time_);
        this->get_parameter("loop", loop_);

        if (mission_file_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "Parâmetro 'mission_file' não definido! Defina o caminho do YAML.");
            throw std::runtime_error("mission_file vazio");
        }

        if (!load_mission(mission_file_)) {
            RCLCPP_FATAL(this->get_logger(),
                "Falha ao carregar missão do arquivo: %s", mission_file_.c_str());
            throw std::runtime_error("missão inválida");
        }

        RCLCPP_INFO(this->get_logger(),
            "Missão carregada: %zu waypoints (loop=%s)",
            waypoints_.size(), loop_ ? "true" : "false");

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MissionFollowerNode::odom_callback, this, std::placeholders::_1));

        // Publisher para o controlador de pose
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);

        // Timer da máquina de estados
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

        // Sobrescreve tolerâncias se definidas no arquivo
        if (mission["position_tolerance"]) pos_tol_ = mission["position_tolerance"].as<double>();
        if (mission["yaw_tolerance"])      yaw_tol_ = mission["yaw_tolerance"].as<double>();
        if (mission["waypoint_timeout"])   wp_timeout_ = mission["waypoint_timeout"].as<double>();
        if (mission["dwell_time"])         dwell_time_ = mission["dwell_time"].as<double>();
        if (mission["loop"])               loop_ = mission["loop"].as<bool>();

        if (mission["frame_id"]) frame_id_ = mission["frame_id"].as<std::string>();

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
            wp.pose.position.x = pos["x"].as<double>();
            wp.pose.position.y = pos["y"].as<double>();
            wp.pose.position.z = pos["z"] ? pos["z"].as<double>() : 0.0;

            auto ori = w["orientation"];
            wp.pose.orientation.x = ori["x"] ? ori["x"].as<double>() : 0.0;
            wp.pose.orientation.y = ori["y"] ? ori["y"].as<double>() : 0.0;
            wp.pose.orientation.z = ori["z"] ? ori["z"].as<double>() : 0.0;
            wp.pose.orientation.w = ori["w"] ? ori["w"].as<double>() : 1.0;

            waypoints_.push_back(wp);
        }

        RCLCPP_INFO(this->get_logger(),
            "Tolerâncias: pos=%.3f m, yaw=%.3f rad, timeout=%.1f s, dwell=%.1f s",
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

    bool reached_waypoint(const geometry_msgs::msg::Pose & target,
                          double & dist_err, double & yaw_err) const
    {
        double dx = target.position.x - current_pose_.position.x;
        double dy = target.position.y - current_pose_.position.y;
        dist_err = std::hypot(dx, dy);

        double yaw_t = yaw_from_quaternion(target.orientation);
        double yaw_c = yaw_from_quaternion(current_pose_.orientation);
        yaw_err = std::fabs(wrap_to_pi(yaw_t - yaw_c));

        return (dist_err <= pos_tol_) && (yaw_err <= yaw_tol_);
    }

    void publish_goal(const Waypoint & wp)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        msg.pose = wp.pose;
        goal_pub_->publish(msg);
    }

    double elapsed_since(rclcpp::Time t) const
    {
        return (this->now() - t).seconds();
    }

    // ---------------- Máquina de estados ----------------
    void state_machine()
    {
        // Sempre precisa do odom para checar chegada
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

                // Timeout opcional
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
                // Nada a fazer. Poderia publicar um goal na pose atual para "segurar".
                break;
        }
    }

    // ---------------- Membros ----------------
    std::string mission_file_;
    std::string frame_id_ = "odom";
    double pos_tol_ = 0.10;
    double yaw_tol_ = 0.15;
    double wp_timeout_ = 60.0;
    double dwell_time_ = 0.5;
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
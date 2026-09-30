#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("no");
    RCLCPP_INFO(node->get_logger(), "Nó 'no' rodando!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
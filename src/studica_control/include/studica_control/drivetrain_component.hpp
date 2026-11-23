#ifndef STUDICA_CONTROL__DRIVETRAIN_COMPONENT_HPP_
#define STUDICA_CONTROL__DRIVETRAIN_COMPONENT_HPP_

#include <array>
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "studica_control/omni_kinematics.hpp"

namespace studica_control {

class Drivetrain : public rclcpp::Node {
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control);
    explicit Drivetrain(rclcpp::Node *control);

private:
    void on_command(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_commands();

    OmniGeometry geometry_;
    std::chrono::steady_clock::time_point last_command_{};
    std::chrono::duration<double> timeout_;
    std::array<double, 3> duties_{};
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> publishers_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace studica_control

#endif  // STUDICA_CONTROL__DRIVETRAIN_COMPONENT_HPP_

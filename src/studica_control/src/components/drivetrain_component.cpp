#include "studica_control/drivetrain_component.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace studica_control {

std::shared_ptr<rclcpp::Node> Drivetrain::initialize(rclcpp::Node *control) {
    return std::make_shared<Drivetrain>(control);
}

Drivetrain::Drivetrain(rclcpp::Node *control) : Node("drivetrain_controller") {
    const auto type = control->declare_parameter<std::string>("drivetrain.type", "");
    const auto titan = control->declare_parameter<std::string>("drivetrain.titan_name", "");
    const auto topic = control->declare_parameter<std::string>("drivetrain.cmd_vel_topic", "cmd_vel");
    const auto motors = control->declare_parameter<std::vector<int64_t>>(
        "drivetrain.motor_indices", std::vector<int64_t>{});
    const auto angles = control->declare_parameter<std::vector<double>>(
        "drivetrain.wheel_angles_deg", std::vector<double>{});
    const auto directions = control->declare_parameter<std::vector<int64_t>>(
        "drivetrain.wheel_directions", std::vector<int64_t>{});
    geometry_.wheel_radius_m = control->declare_parameter<double>("drivetrain.wheel_radius_m", 0.0);
    geometry_.wheel_distance_m = control->declare_parameter<double>("drivetrain.wheel_distance_m", 0.0);
    geometry_.max_wheel_speed_rad_s = control->declare_parameter<double>("drivetrain.max_wheel_speed_rad_s", 0.0);
    geometry_.max_duty = control->declare_parameter<double>("drivetrain.max_duty", 0.0);
    const double timeout = control->declare_parameter<double>("drivetrain.cmd_timeout_sec", 0.3);
    const int rate = control->declare_parameter<int>("drivetrain.publish_rate_hz", 50);

    if (type != "omni3") {
        throw std::invalid_argument("drivetrain.type must be omni3; other types are not implemented");
    }
    const auto titan_names = control->get_parameter("titan.sensors").as_string_array();
    if (std::find(titan_names.begin(), titan_names.end(), titan) == titan_names.end() ||
        control->get_parameter("titan." + titan + ".topic_timeout_sec").as_double() <= 0.0) {
        throw std::invalid_argument("drivetrain Titan must exist and have topic_timeout_sec > 0");
    }
    if (titan.empty() || topic.empty() || motors.size() != 3 || angles.size() != 3 ||
        directions.size() != 3 || std::set<int64_t>(motors.begin(), motors.end()).size() != 3 ||
        !std::all_of(motors.begin(), motors.end(), [](int64_t n) { return n >= 0 && n <= 3; }) ||
        !std::all_of(directions.begin(), directions.end(), [](int64_t n) { return n == -1 || n == 1; }) ||
        !std::all_of(angles.begin(), angles.end(), [](double n) { return std::isfinite(n); }) ||
        !std::isfinite(geometry_.wheel_radius_m) || geometry_.wheel_radius_m <= 0 ||
        !std::isfinite(geometry_.wheel_distance_m) || geometry_.wheel_distance_m <= 0 ||
        !std::isfinite(geometry_.max_wheel_speed_rad_s) || geometry_.max_wheel_speed_rad_s <= 0 ||
        !std::isfinite(geometry_.max_duty) || geometry_.max_duty <= 0 || geometry_.max_duty > 1 ||
        !std::isfinite(timeout) || timeout <= 0 || rate < 1 || rate > 1000) {
        throw std::invalid_argument("invalid drivetrain configuration; see params_template.yaml");
    }
    timeout_ = std::chrono::duration<double>(timeout);
    constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
    for (size_t i = 0; i < 3; ++i) {
        geometry_.wheel_angles_rad[i] = std::remainder(angles[i], 360.0) * deg_to_rad;
        geometry_.wheel_directions[i] = static_cast<double>(directions[i]);
        publishers_[i] = create_publisher<std_msgs::msg::Float64>(
            "/" + titan + "/m_" + std::to_string(motors[i]) + "/cmd", 1);
    }
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        topic, 1, std::bind(&Drivetrain::on_command, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / rate),
        std::bind(&Drivetrain::publish_commands, this));
    RCLCPP_INFO(get_logger(), "omni3 drivetrain listening on %s", topic.c_str());
}

void Drivetrain::on_command(const geometry_msgs::msg::Twist::SharedPtr msg) {
    try {
        duties_ = omni_duties(geometry_, msg->linear.x, msg->linear.y, msg->angular.z);
        last_command_ = std::chrono::steady_clock::now();
    } catch (const std::invalid_argument &error) {
        duties_.fill(0.0);
        last_command_ = {};
        RCLCPP_ERROR(get_logger(), "Stopping drivetrain: %s", error.what());
    }
}

void Drivetrain::publish_commands() {
    if (last_command_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - last_command_ > timeout_) {
        duties_.fill(0.0);
    }
    for (size_t i = 0; i < duties_.size(); ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = duties_[i];
        publishers_[i]->publish(msg);
    }
}

}  // namespace studica_control

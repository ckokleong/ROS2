#include "studica_control/wheel_odometry_component.hpp"
#include "studica_control/wheel_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace studica_control {

std::shared_ptr<rclcpp::Node> WheelOdometry::initialize(rclcpp::Node *control) {
    return std::make_shared<WheelOdometry>(control);
}

WheelOdometry::WheelOdometry(rclcpp::Node *control) : Node("wheel_odometry") {
    if (control->get_parameter("use_sim_time").as_bool()) {
        throw std::invalid_argument("wheel odometry currently supports hardware system time only");
    }
    if (!control->get_parameter("drivetrain.enabled").as_bool() ||
        control->get_parameter("drivetrain.type").as_string() != "omni3" ||
        !control->get_parameter("imu.enabled").as_bool()) {
        throw std::invalid_argument("odometry requires enabled omni3 drivetrain and IMU");
    }
    if (control->get_parameter("imu.gyro_variance").as_double() <= 0.0) {
        throw std::invalid_argument("odometry requires a positive imu.gyro_variance");
    }
    const auto titan = control->get_parameter("drivetrain.titan_name").as_string();
    const auto motors = control->get_parameter("drivetrain.motor_indices").as_integer_array();
    const auto angles = control->get_parameter("drivetrain.wheel_angles_deg").as_double_array();
    distance_ = control->get_parameter("drivetrain.wheel_distance_m").as_double();
    const auto dirs = control->declare_parameter<std::vector<int64_t>>(
        "odometry.encoder_directions", std::vector<int64_t>{});
    base_frame_ = control->declare_parameter<std::string>("odometry.base_frame", "base_footprint");
    const auto topic = control->declare_parameter<std::string>("odometry.wheel_twist_topic", "/wheel_twist");
    max_skew_ = control->declare_parameter<double>("odometry.max_sample_skew_sec", 0.03);
    max_interval_ = control->declare_parameter<double>("odometry.max_sample_interval_sec", 0.3);
    max_speed_ = control->declare_parameter<double>("odometry.max_wheel_speed_m_s", 0.0);
    const auto variances = control->declare_parameter<std::vector<double>>(
        "odometry.twist_variances", std::vector<double>{});

    if (dirs.size() != 3 || variances.size() != 3 || base_frame_.empty() || topic.empty() ||
        !std::all_of(dirs.begin(), dirs.end(), [](int64_t n) { return n == -1 || n == 1; }) ||
        !std::all_of(variances.begin(), variances.end(), [](double n) { return std::isfinite(n) && n > 0.0; }) ||
        !std::isfinite(max_skew_) || max_skew_ <= 0 ||
        !std::isfinite(max_interval_) || max_interval_ <= max_skew_ ||
        !std::isfinite(max_speed_) || max_speed_ <= 0) {
        throw std::invalid_argument("invalid odometry parameters; see params_template.yaml");
    }
    constexpr double rad = 3.14159265358979323846 / 180.0;
    for (size_t i = 0; i < 3; ++i) {
        angles_[i] = std::remainder(angles[i], 360.0) * rad;
        directions_[i] = static_cast<double>(dirs[i]);
        variances_[i] = variances[i];
        const auto prefix = "titan." + titan + ".m_" + std::to_string(motors[i]);
        if (control->get_parameter(prefix + ".encoder_mode").as_string() != "quadrature" ||
            !std::isfinite(control->get_parameter(prefix + ".dist_per_tick").as_double()) ||
            control->get_parameter(prefix + ".dist_per_tick").as_double() <= 0.0) {
            throw std::invalid_argument("odometry requires quadrature encoders with positive dist_per_tick");
        }
    }
    (void)omni_body_velocity(angles_, distance_, {0.0, 0.0, 0.0});
    publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic, 10);
    for (size_t i = 0; i < 3; ++i) {
        const auto encoder_topic = "/" + titan + "/m_" + std::to_string(motors[i]) + "/encoder_stamped";
        subscriptions_[i] = create_subscription<Encoder>(encoder_topic, 10,
            [this, i](Encoder::SharedPtr msg) { on_encoder(i, msg); });
    }
    RCLCPP_INFO(get_logger(), "wheel odometry publishes %s in %s", topic.c_str(), base_frame_.c_str());
}

void WheelOdometry::on_encoder(size_t wheel, Encoder::SharedPtr msg) {
    if (!std::isfinite(msg->distance)) {
        RCLCPP_ERROR(get_logger(), "invalid encoder sample; resetting baseline");
        pending_ = {};
        previous_ = {};
        return;
    }
    pending_[wheel] = msg;
    if (std::any_of(pending_.begin(), pending_.end(), [](const Encoder::SharedPtr &p) { return !p; })) return;

    auto stamp = [](const Encoder::SharedPtr &p) { return rclcpp::Time(p->header.stamp).seconds(); };
    auto minmax = std::minmax_element(pending_.begin(), pending_.end(),
        [&](const Encoder::SharedPtr &a, const Encoder::SharedPtr &b) { return stamp(a) < stamp(b); });
    if (stamp(*minmax.second) - stamp(*minmax.first) > max_skew_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "encoder sample skew too high");
        *minmax.first = nullptr;
        return;
    }
    const auto samples = pending_;
    pending_ = {};
    if (std::any_of(previous_.begin(), previous_.end(), [](const Encoder::SharedPtr &p) { return !p; })) {
        previous_ = samples;
        return;
    }
    std::array<double, 3> velocity{};
    for (size_t i = 0; i < 3; ++i) {
        const double dt = stamp(samples[i]) - stamp(previous_[i]);
        const double delta = samples[i]->distance - previous_[i]->distance;
        if (samples[i]->epoch != previous_[i]->epoch || dt <= 0.0 || dt > max_interval_ ||
            !std::isfinite(delta) || std::abs(delta) > max_speed_ * dt) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "encoder reset, stale sample, or implausible wheel speed; resetting baseline");
            previous_ = samples;
            return;
        }
        velocity[i] = directions_[i] * delta / dt;
    }
    previous_ = samples;
    const auto body = omni_body_velocity(angles_, distance_, velocity);
    geometry_msgs::msg::TwistWithCovarianceStamped output;
    const auto latest = std::max_element(samples.begin(), samples.end(),
        [&](const Encoder::SharedPtr &a, const Encoder::SharedPtr &b) { return stamp(a) < stamp(b); });
    output.header.stamp = (*latest)->header.stamp;
    output.header.frame_id = base_frame_;
    output.twist.twist.linear.x = body[0];
    output.twist.twist.linear.y = body[1];
    output.twist.twist.angular.z = body[2];
    output.twist.covariance[0] = variances_[0];
    output.twist.covariance[7] = variances_[1];
    output.twist.covariance[35] = variances_[2];
    publisher_->publish(output);
}

}  // namespace studica_control

#include "studica_control/wheel_odometry_component.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Encoder = studica_control::msg::EncoderDistanceStamped;
using Twist = geometry_msgs::msg::TwistWithCovarianceStamped;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"odometry.encoder_directions", std::vector<int64_t>{1, -1, 1}},
        {"odometry.max_wheel_speed_m_s", 2.0},
        {"odometry.twist_variances", std::vector<double>{0.02, 0.02, 0.05}},
    });
    auto config = std::make_shared<rclcpp::Node>("test_control_configured", options);
    config->declare_parameter("drivetrain.enabled", true);
    config->declare_parameter("drivetrain.type", "omni3");
    config->declare_parameter("imu.enabled", true);
    config->declare_parameter("imu.gyro_variance", 0.01);
    config->declare_parameter("drivetrain.titan_name", "test_titan");
    config->declare_parameter("drivetrain.motor_indices", std::vector<int64_t>{3, 0, 1});
    config->declare_parameter("drivetrain.wheel_angles_deg", std::vector<double>{60.0, 300.0, 180.0});
    config->declare_parameter("drivetrain.wheel_distance_m", 0.214);
    for (int i : {3, 0, 1}) {
        const auto prefix = "titan.test_titan.m_" + std::to_string(i);
        config->declare_parameter(prefix + ".encoder_mode", "quadrature");
        config->declare_parameter(prefix + ".dist_per_tick", 0.0002399828);
    }
    auto wheel = std::make_shared<studica_control::WheelOdometry>(config.get());
    auto test = std::make_shared<rclcpp::Node>("odometry_test");
    std::array<rclcpp::Publisher<Encoder>::SharedPtr, 3> pubs;
    const std::array<int, 3> motors{3, 0, 1};
    for (size_t i = 0; i < 3; ++i) {
        pubs[i] = test->create_publisher<Encoder>("/test_titan/m_" + std::to_string(motors[i]) +
                                                 "/encoder_stamped", 10);
    }
    std::vector<Twist> output;
    auto sub = test->create_subscription<Twist>("/wheel_twist", 10,
        [&](Twist::SharedPtr msg) { output.push_back(*msg); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test);
    executor.add_node(wheel);
    auto spin = [&](int ms) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end) {
            executor.spin_some();
            std::this_thread::sleep_for(2ms);
        }
    };
    spin(500);
    for (const auto &pub : pubs) {
        if (pub->get_subscription_count() != 1) throw std::runtime_error("encoder subscription missing");
    }
    auto publish = [&](double time, const std::array<double, 3> &dist, uint64_t epoch) {
        for (size_t i = 0; i < 3; ++i) {
            Encoder msg;
            msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
            msg.distance = dist[i];
            msg.epoch = epoch;
            pubs[i]->publish(msg);
        }
        spin(100);
    };
    constexpr double pi = 3.14159265358979323846;
    std::array<double, 3> d{-std::sin(pi / 3.0) * 0.01,
                            -std::sin(pi / 3.0) * 0.01, 0.0};
    publish(10.0, {0.0, 0.0, 0.0}, 0);
    if (!output.empty()) throw std::runtime_error("baseline should not publish");
    publish(10.1, d, 0);
    if (output.size() != 1 || std::abs(output.back().twist.twist.linear.x - 0.1) > 1e-6 ||
        std::abs(output.back().twist.twist.linear.y) > 1e-6 ||
        output.back().header.frame_id != "base_footprint") {
        throw std::runtime_error("incorrect body velocity");
    }
    publish(10.2, {0.0, 0.0, 0.0}, 1);
    if (output.size() != 1) throw std::runtime_error("reset should rebaseline");
    publish(10.3, d, 1);
    if (output.size() != 2) throw std::runtime_error("velocity did not resume after reset");
    publish(10.4, {10.0, 0.0, 0.0}, 1);
    if (output.size() != 2) throw std::runtime_error("implausible speed was accepted");
    publish(11.0, d, 1);
    if (output.size() != 2) throw std::runtime_error("stale interval was accepted");
    publish(11.1, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 1);
    if (output.size() != 2) throw std::runtime_error("invalid sample was accepted");
    publish(11.2, {0.0, 0.0, 0.0}, 1);
    if (output.size() != 2) throw std::runtime_error("invalid sample must reset baseline");
    publish(11.3, d, 1);
    if (output.size() != 3) throw std::runtime_error("did not recover from invalid sample");
    executor.remove_node(wheel);
    executor.remove_node(test);
    rclcpp::shutdown();
}

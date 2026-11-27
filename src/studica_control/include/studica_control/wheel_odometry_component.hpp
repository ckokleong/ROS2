#ifndef STUDICA_CONTROL__WHEEL_ODOMETRY_COMPONENT_HPP_
#define STUDICA_CONTROL__WHEEL_ODOMETRY_COMPONENT_HPP_

#include <array>
#include <memory>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "studica_control/msg/encoder_distance_stamped.hpp"

namespace studica_control {

class WheelOdometry : public rclcpp::Node {
public:
    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control);
    explicit WheelOdometry(rclcpp::Node *control);

private:
    using Encoder = studica_control::msg::EncoderDistanceStamped;
    void on_encoder(size_t wheel, Encoder::SharedPtr msg);
    std::array<double, 3> angles_{};
    std::array<double, 3> directions_{};
    double distance_{};
    double max_skew_{};
    double max_interval_{};
    double max_speed_{};
    std::array<double, 3> variances_{};
    std::string base_frame_;
    std::array<Encoder::SharedPtr, 3> pending_{};
    std::array<Encoder::SharedPtr, 3> previous_{};
    std::array<rclcpp::Subscription<Encoder>::SharedPtr, 3> subscriptions_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
};

}  // namespace studica_control

#endif  // STUDICA_CONTROL__WHEEL_ODOMETRY_COMPONENT_HPP_

/*
 * imu_component.cpp
 *
 * ros2 component for the onboard imu (inertial measurement unit).
 * reads orientation, rotation rate, and linear acceleration from the
 * navx sensor built into the vmx-pi board.
 *
 * topic:   <topic> (sensor_msgs/Imu) — published at 20hz
 *            orientation as a quaternion (x, y, z, w)
 *            angular velocity in radians per second (x, y, z)
 *            linear acceleration in meters per second squared (x, y, z)
 *
 * service: get_imu_data (studica_control/SetData)
 *   returns the current pitch, yaw, and roll as a string
 *   (no specific params field needed — any call returns the values)
 */

#include "studica_control/imu_component.hpp"
#include <cmath>
#include <stdexcept>

namespace studica_control {


// reads imu parameters from params.yaml and creates the imu node
std::shared_ptr<rclcpp::Node> Imu::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("imu.name");
    control->declare_parameter<std::string>("imu.topic");
    control->declare_parameter<std::string>("imu.frame_id", "imu_link");
    control->declare_parameter<double>("imu.gyro_variance", -1.0);

    std::string name     = control->get_parameter("imu.name").as_string();
    std::string topic    = control->get_parameter("imu.topic").as_string();
    std::string frame_id = control->get_parameter("imu.frame_id").as_string();
    double gyro_variance = control->get_parameter("imu.gyro_variance").as_double();
    if (!std::isfinite(gyro_variance) || (gyro_variance <= 0.0 && gyro_variance != -1.0)) {
        throw std::invalid_argument("imu.gyro_variance must be positive or -1 (unavailable)");
    }

    return std::make_shared<Imu>(vmx, name, topic, frame_id, gyro_variance);
}


// composable node constructor — used when loading as a plugin
Imu::Imu(const rclcpp::NodeOptions &options) : Node("imu", options) {}


// main constructor — connects to the imu and sets up the publisher,
// service, and periodic timer. zeros the yaw on startup.
Imu::Imu(std::shared_ptr<VMXPi> vmx, const std::string &name, const std::string &topic, const std::string &frame_id,
         double gyro_variance)
    : rclcpp::Node(name), vmx_(vmx), frame_id_(frame_id), gyro_variance_(gyro_variance) {

    imu_ = std::make_shared<studica_driver::Imu>(vmx_);
    imu_->ZeroYaw();

    // service for reading pitch, yaw, and roll on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes full imu data at 20hz
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Imu::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "imu component ready.");
}

Imu::~Imu() {}


void Imu::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (request->params == "zero_yaw") {
            imu_->ZeroYaw();
            response->success = true;
            response->message = "yaw zeroed";
            RCLCPP_INFO(this->get_logger(), "yaw zeroed");

        } else {
            // default — return current pitch, yaw, roll
            float pitch = imu_->GetPitch();
            float yaw   = imu_->GetYaw();
            float roll  = imu_->GetRoll();
            response->success = true;
            response->message = "pitch: " + std::to_string(pitch)
                              + ", yaw: "  + std::to_string(yaw)
                              + ", roll: " + std::to_string(roll);
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "imu error: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "imu error: %s", e.what());
    }
}


// reads orientation, angular velocity, and linear acceleration from the
// imu and publishes them as a standard ros2 imu message.
// units: orientation as quaternion, angular velocity in rad/s, acceleration in m/s²
// Orientation and acceleration are marked unavailable; gyro variance is configurable.
void Imu::publish_data() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // NavX native frame: X=right, Y=forward, Z=up, CW+ yaw.
    // Remapped to ROS convention: X=forward, Y=left, Z=up, CCW+ yaw.
    // Quaternion: swap X/Y axes. Z is already CCW+ in the NavX quaternion
    // (the NavX SDK internally uses CCW+ for quaternion Z despite reporting
    // Euler yaw as CW+), so no negation needed on Z.
    //
    // The NavX reports quaternions in the negative hemisphere (w<0 at rest).
    // q and -q encode the same rotation, but ROS convention requires w>=0
    // (canonical/shorter-arc form). Normalize here so EKFs, nav2, and tf2
    // all receive a standard quaternion without relying on sign-handling quirks.
    {
        float qx =  imu_->GetQuaternionY();
        float qy = -imu_->GetQuaternionX();
        float qz =  imu_->GetQuaternionZ();
        float qw =  imu_->GetQuaternionW();
        if (qw < 0.0f) { qx = -qx; qy = -qy; qz = -qz; qw = -qw; }
        msg.orientation.x = qx;
        msg.orientation.y = qy;
        msg.orientation.z = qz;
        msg.orientation.w = qw;
    }

    // raw gyro in deg/s — remap axes and negate Z for CCW+, convert to rad/s
    msg.angular_velocity.x =  imu_->GetRawGyroY() * (M_PI / 180.0);
    msg.angular_velocity.y = -imu_->GetRawGyroX() * (M_PI / 180.0);
    msg.angular_velocity.z = -imu_->GetRawGyroZ() * (M_PI / 180.0);

    // raw body-frame acceleration in g's — remap axes, convert to m/s²
    // (includes gravity; robot_localization removes it via imu0_remove_gravitational_acceleration)
    msg.linear_acceleration.x =  imu_->GetRawAccelY() * 9.80665;
    msg.linear_acceleration.y = -imu_->GetRawAccelX() * 9.80665;
    msg.linear_acceleration.z =  imu_->GetRawAccelZ() * 9.80665;

    // -1 in the first covariance element means the measurement is unavailable.
    msg.orientation_covariance[0]         = -1.0;
    if (gyro_variance_ > 0.0 && std::isfinite(msg.angular_velocity.x) &&
        std::isfinite(msg.angular_velocity.y) && std::isfinite(msg.angular_velocity.z)) {
        msg.angular_velocity_covariance[0] = gyro_variance_;
        msg.angular_velocity_covariance[4] = gyro_variance_;
        msg.angular_velocity_covariance[8] = gyro_variance_;
    } else {
        msg.angular_velocity_covariance[0] = -1.0;
        if (gyro_variance_ > 0.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU gyro sample is non-finite");
        }
    }
    msg.linear_acceleration_covariance[0] = -1.0;

    publisher_->publish(msg);
}


void Imu::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("vmx error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Imu)

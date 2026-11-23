#ifndef STUDICA_CONTROL__OMNI_KINEMATICS_HPP_
#define STUDICA_CONTROL__OMNI_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace studica_control {

struct OmniGeometry {
    std::array<double, 3> wheel_angles_rad;
    std::array<double, 3> wheel_directions;
    double wheel_radius_m;
    double wheel_distance_m;
    double max_wheel_speed_rad_s;
    double max_duty;
};

inline std::array<double, 3> omni_duties(
    const OmniGeometry &geometry, double vx, double vy, double wz) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
        throw std::invalid_argument("non-finite velocity command");
    }

    std::array<double, 3> duties{};
    double peak = 1.0;
    for (size_t i = 0; i < duties.size(); ++i) {
        const double tangential_speed =
            -std::sin(geometry.wheel_angles_rad[i]) * vx +
            std::cos(geometry.wheel_angles_rad[i]) * vy +
            geometry.wheel_distance_m * wz;
        duties[i] = geometry.wheel_directions[i] * tangential_speed /
                    geometry.wheel_radius_m / geometry.max_wheel_speed_rad_s;
        if (!std::isfinite(duties[i])) {
            throw std::invalid_argument("wheel speed calculation overflow");
        }
        peak = std::max(peak, std::abs(duties[i]));
    }
    for (double &duty : duties) {
        duty = duty / peak * geometry.max_duty;
    }
    return duties;
}

}  // namespace studica_control

#endif  // STUDICA_CONTROL__OMNI_KINEMATICS_HPP_

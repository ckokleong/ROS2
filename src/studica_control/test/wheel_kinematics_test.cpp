#include "studica_control/wheel_kinematics.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

int main() {
    constexpr double pi = 3.14159265358979323846;
    const std::array<double, 3> angles{pi / 3.0, 5.0 * pi / 3.0, pi};
    const double radius = 0.214;
    const std::array<double, 3> expected{0.3, -0.2, 0.4};
    std::array<double, 3> wheel{};
    for (size_t i = 0; i < 3; ++i) {
        wheel[i] = -std::sin(angles[i]) * expected[0] +
                   std::cos(angles[i]) * expected[1] + radius * expected[2];
    }
    const auto actual = studica_control::omni_body_velocity(angles, radius, wheel);
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(actual[i] - expected[i]) > 1e-10) return 1;
    }
    try {
        studica_control::omni_body_velocity({0.0, 0.0, 0.0}, radius, wheel);
        return 2;
    } catch (const std::invalid_argument &) {
        return 0;
    }
}

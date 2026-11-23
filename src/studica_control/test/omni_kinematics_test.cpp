#include "studica_control/omni_kinematics.hpp"

#include <cassert>
#include <cmath>
#include <limits>

int main() {
    constexpr double pi = 3.14159265358979323846;
    studica_control::OmniGeometry geometry{
        {0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0},
        {1.0, 1.0, 1.0}, 0.05, 0.2, 20.0, 0.5};
    auto close = [](double a, double b) { return std::abs(a - b) < 1e-9; };
    auto forward = studica_control::omni_duties(geometry, 1.0, 0.0, 0.0);
    assert(close(forward[0], 0.0));
    assert(close(forward[1], -std::sqrt(3.0) / 4.0));
    assert(close(forward[2], std::sqrt(3.0) / 4.0));
    auto strafe = studica_control::omni_duties(geometry, 0.0, 1.0, 0.0);
    assert(close(strafe[0], 0.5));
    assert(close(strafe[1], -0.25));
    assert(close(strafe[2], -0.25));
    auto rotate = studica_control::omni_duties(geometry, 0.0, 0.0, 1.0);
    assert(close(rotate[0], 0.1) && close(rotate[1], 0.1) && close(rotate[2], 0.1));
    geometry.wheel_directions[0] = -1.0;
    auto saturated = studica_control::omni_duties(geometry, 0.0, 3.0, 0.0);
    assert(close(saturated[0], -0.5));
    assert(close(saturated[1], -0.25));
    assert(close(saturated[2], -0.25));
    bool rejected = false;
    try {
        studica_control::omni_duties(geometry, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    assert(rejected);
}

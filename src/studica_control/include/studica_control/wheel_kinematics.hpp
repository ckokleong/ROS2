#ifndef STUDICA_CONTROL__WHEEL_KINEMATICS_HPP_
#define STUDICA_CONTROL__WHEEL_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace studica_control {

// Each row projects body velocity onto the tangential direction of one wheel.
inline std::array<double, 3> omni_body_velocity(
    const std::array<double, 3> &angles, double distance,
    const std::array<double, 3> &wheel_velocity) {
    std::array<std::array<double, 4>, 3> matrix{};
    for (size_t i = 0; i < 3; ++i) {
        matrix[i] = {-std::sin(angles[i]), std::cos(angles[i]), distance, wheel_velocity[i]};
    }
    for (size_t col = 0; col < 3; ++col) {
        size_t pivot = col;
        for (size_t row = col + 1; row < 3; ++row) {
            if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) pivot = row;
        }
        if (std::abs(matrix[pivot][col]) < 1e-9) {
            throw std::invalid_argument("omni3 wheel geometry is singular");
        }
        std::swap(matrix[pivot], matrix[col]);
        const double divisor = matrix[col][col];
        for (size_t j = col; j < 4; ++j) matrix[col][j] /= divisor;
        for (size_t row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double factor = matrix[row][col];
            for (size_t j = col; j < 4; ++j) matrix[row][j] -= factor * matrix[col][j];
        }
    }
    return {matrix[0][3], matrix[1][3], matrix[2][3]};
}

}  // namespace studica_control

#endif  // STUDICA_CONTROL__WHEEL_KINEMATICS_HPP_

#pragma once
#include <Eigen/Dense>
#include <vector>
namespace ARDL {
    template <typename T> using SpatialInertia = Eigen::Matrix<T, 6, 6>;
    template <typename T> using Jacobian = Eigen::Matrix<T, 6, Eigen::Dynamic>;
    template <typename T> using Regressor = Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic>;
    template <typename T> using LinkRegressor = Eigen::Matrix <T, 6, 10>;
    template <typename T> using aligned_vector = std::vector<T, Eigen::aligned_allocator<T> >;
    template <typename T> using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    template <typename T> using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    template <typename T> using Gravity = Eigen::Matrix<T, 6, 1>;
}

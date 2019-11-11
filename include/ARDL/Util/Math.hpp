#pragma once
#include <Eigen/Dense>

namespace ARDL {
    namespace Util {
        namespace Math {
            /**
             * @brief Turns a vector into a skew-symmetric matrix
             *
             * @tparam Derived Type of vector to allow for block/segment calls
             * @tparam Derived2 Type of matrix to allow for block/segment calls
             * @param vector Input 3x1 vector
             * @param matrix Output 3x3 matrix
             */
            template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &matrix) {
                typedef typename Derived::Scalar Scalar;
                const_cast< Eigen::MatrixBase<Derived2> & >(matrix).setZero();
                const_cast<Scalar &>(matrix(2, 1)) = vector(0);
                const_cast<Scalar &>(matrix(1, 2)) = -vector(0);
                const_cast<Scalar &>(matrix(0, 2)) = vector(1);
                const_cast<Scalar &>(matrix(2, 0)) = -vector(1);
                const_cast<Scalar &>(matrix(0, 1)) = -vector(2);
                const_cast<Scalar &>(matrix(1, 0)) = vector(2);
            }

            /**
             * @brief Turns a vector into the double tilde matrix (On the closed form computation of the
               dynamic matrices and their differentiation eq. (12))
             *
             * @tparam Derived Type of vector to allow for block/segment calls
             * @tparam Derived2 Type of matrix to allow for block/segment calls
             * @param vector Input 3x1 vector
             * @param matrix Output 3x6 matrix
             */
            template<typename Derived, typename Derived2> static void tildeMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &matrix) {
                typedef typename Derived::Scalar Scalar;
                const_cast< Eigen::MatrixBase<Derived2> & >(matrix).setZero();
                const_cast<Scalar &>(matrix(0, 0)) = vector(0);
                const_cast<Scalar &>(matrix(1, 1)) = vector(1);
                const_cast<Scalar &>(matrix(2, 2)) = vector(2);
                const_cast<Scalar &>(matrix(1, 3)) = vector(0);
                const_cast<Scalar &>(matrix(2, 4)) = vector(0);
                const_cast<Scalar &>(matrix(0, 3)) = vector(1);
                const_cast<Scalar &>(matrix(2, 5)) = vector(1);
                const_cast<Scalar &>(matrix(0, 4)) = vector(2);
                const_cast<Scalar &>(matrix(1, 5)) = vector(2);
            }

            /**
             * @brief Analytic sign function
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Steepness of output change around 0
             * @return int The sign of the input value
             */
            template <typename T> static int sgn(const T &val, const T &eps = T(1e-50)) {
                return val / std::sqrt(std::pow(val, 2) + eps);
            }

            /**
             * @brief Analytic derivative of sign function
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Steepness of output change around 0
             * @return int The derivative of the sign of the input value
             */
            template <typename T> static T sgnDeriv(const T &val, const T &eps = T(1e-50)) {
                return eps / std::pow(std::pow(val, 2) + eps, 1.5);
            }

            /**
             * @brief Branchless sign function using bool comparisons
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Deadzone around 0 for the sign function
             * @return int The sign of the input value
             */
            template <typename T> static int sgnIF(const T &val, const T &eps = T(0)) {
                return (T(eps) < val) - (val < T(-eps));
            }

            /**
             * @brief Branchless deadzone function using bool comparisons and inversions. Deadzone is a function that calculates a deadzone around zero for a number. If the number lies within the bounds this function returns 0 otherwise it returns 1.
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Deadzone around 0
             * @return int 0 if input is between the \pm eps bounds otherwise 1
             */
            template <typename T> static int deadzone(const T &val, const T &eps = T(0)) {
                return !((T(eps) > val) && (val > T(-eps)));
            }
        }
    }
}

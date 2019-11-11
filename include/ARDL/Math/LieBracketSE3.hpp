#pragma once
#include <ostream>

#include "ARDL/Util/Math.hpp"

namespace ARDL {
    namespace Math {
        /**
         * @brief SE3 Lie Bracket Matrix (velocity SE3 Lie Group)
         *
         * @tparam T Base type
         */
        template <typename T> class LieBracketSE3 {
        private:
            /**
             * @brief Velocity vector
             *
             */
            Eigen::Matrix<T, 6, 1> m_velocity;
            /**
             * @brief SE3 Velocity Matrix
             *
             */
            Eigen::Matrix<T, 6, 6> m_matrix;
        public:
            LieBracketSE3() {
                m_velocity.setZero();
                m_matrix.setZero();
            }
            template <typename VectorD> LieBracketSE3(const Eigen::MatrixBase<VectorD> &velocity) {
                m_velocity = velocity;
                calcMatrix();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            void calcMatrix() {
                ARDL::Util::Math::skewMatrix(m_velocity.template tail<3>(), m_matrix.template block<3, 3>(0, 0));
                m_matrix.template block<3, 3>(3, 3) = m_matrix.template block<3, 3>(0, 0);
                ARDL::Util::Math::skewMatrix(m_velocity.template head<3>(), m_matrix.template block<3, 3>(0, 3));
            }

            const Eigen::Matrix<T, 6, 6> &getMatrix() const {
                return m_matrix;
            }

            Eigen::Matrix<T, 6, 1> &getVelocity() {
                return m_velocity;
            }

            void setIdentity() {
                m_velocity.setZero();
                m_matrix.setZero();
            }
            template <typename VectorD> void setVelocity(const Eigen::MatrixBase<VectorD> &velocity) {
                m_velocity = velocity;
                calcMatrix();
            }
        };
    }
}

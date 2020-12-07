#pragma once
#include <ostream>

#include "ARDL/Util/Math.hpp"
#include "ARDL/Math/SpatialInertia.hpp"
#include <Eigen/Geometry>

namespace ARDL {
    namespace Math {

template <typename T> class Pose;
        /**
         * @brief SE3 Lie Bracket Matrix (velocity SE3 Lie Group)
         *
         * @tparam T Base type
         */
        template<typename T>
        class Motion {
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
            mutable Eigen::Matrix<T, 6, 6> m_matrix;

            mutable bool cache= false;

            mutable Eigen::Matrix<T, 3, 3> mt_skew;

             public:
            constexpr Motion() {
                m_velocity= Eigen::Matrix<T, 6, 1>::Zero();
                m_matrix= Eigen::Matrix<T, 6, 6>::Zero();
            }
            template<typename VectorD>
            Motion(const Eigen::MatrixBase<VectorD> &velocity) {
                cache= false;
                m_velocity= velocity;
                calcMatrix();
            }

            Motion &operator=(const Motion &in) {
                cache= false;
                this->m_velocity= in.getVelocityConst();
                return *this;
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            void calcMatrix() const {
                ARDL::Util::Math::skewMatrix(m_velocity.template tail<3>(), m_matrix.template block<3, 3>(0, 0));
                m_matrix.template block<3, 3>(3, 3)= m_matrix.template block<3, 3>(0, 0);
                ARDL::Util::Math::skewMatrix(m_velocity.template head<3>(), m_matrix.template block<3, 3>(0, 3));
            }

            const Eigen::Matrix<T, 6, 6> &getMatrix() const {
                if(!cache) {
                    calcMatrix();
                    cache= true;
                }
                return m_matrix;
            }

            inline Eigen::Matrix<T, 6, 1> &getVelocity() { return m_velocity; }

            template<typename D>
            inline void setVelocity(const Eigen::MatrixBase<D> &in) {
                cache= false;
                m_velocity= in;
            }

            inline const Eigen::Matrix<T, 6, 1> &getVelocityConst() const { return m_velocity; }

            template<typename Derived, typename Derived3>
            void apply(const Eigen::MatrixBase<Derived> &in, const Eigen::MatrixBase<Derived3> &out) const {
                skewMatrix(m_velocity.template tail<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<Derived3> &>(out)
                    .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
                    .noalias()= mt_skew * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<Eigen::MatrixBase<Derived3> &>(out)
                    .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
                    .noalias()= mt_skew * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());

                skewMatrix(m_velocity.template head<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<Derived3> &>(out)
                    .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
                    .noalias()+= mt_skew * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());

            }

            template<typename Derived, typename Derived2, typename Derived3>
            void apply(const Eigen::MatrixBase<Derived> &in, const Eigen::MatrixBase<Derived2> &out,
                       const Eigen::MatrixBase<Derived3> &buffer) const {
                skewMatrix(m_velocity.template tail<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
                    .noalias()= mt_skew * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
                    .noalias()= mt_skew * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());

                skewMatrix(m_velocity.template head<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
                    .noalias()+= mt_skew * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<Eigen::MatrixBase<Derived2> &>(out)= buffer;
            }

            template<typename D>
            void applyTranspose(SpatialInertia<T> &si, const Eigen::MatrixBase<D> &out) {
                // tilde w
                skewMatrix(m_velocity.template tail<3>(),
                           const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(0, 0));
                // tilde w
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 3)= out.template block<3, 3>(0, 0);
                //-tilde w * I
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 3)*= -si.getInertia();

                // tilde w * m
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(0, 3)= out.template block<3, 3>(0, 0);

                // tilde r
                skewMatrix(si.getMomentMass(), mt_skew);

                // tilde w * m * tilde r
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(0, 3)*= mt_skew;

                //-tilde w * m
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(0, 0)*= -si.getMass();

                // tilde v
                skewMatrix(m_velocity.template head<3>(),
                           const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 0));

                // tilde v m tilde r - \tilde w I
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 3).noalias()+=
                    out.template block<3, 3>(3, 0) * mt_skew;

                //-tilde v m
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 0)*= -si.getMass();

                //-tilde v m - tilde w m tilde r
                const_cast<Eigen::MatrixBase<D> &>(out).template block<3, 3>(3, 0)-= out.template block<3, 3>(0, 3);
            }

            template<typename DI, typename DO>
            void applyTransposeTo(const Eigen::MatrixBase<DI> &in, const Eigen::MatrixBase<DO> &out) {
                skewMatrix(m_velocity.template head<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<DO> &>(out).template block<3, -1>(3, 0, 3, in.cols()).noalias()=
                    -mt_skew * in.template block<3, -1>(0, 0, 3, in.cols());
                skewMatrix(m_velocity.template tail<3>(), mt_skew);
                const_cast<Eigen::MatrixBase<DO> &>(out).template block<3, -1>(3, 0, 3, in.cols()).noalias()-=
                    mt_skew * in.template block<3, -1>(3, 0, 3, in.cols());

                const_cast<Eigen::MatrixBase<DO> &>(out).template block<3, -1>(0, 0, 3, in.cols()).noalias()=
                    -mt_skew * in.template block<3, -1>(0, 0, 3, in.cols());
            }

            void setIdentity() { m_velocity.setZero(); }

            // TODO add safetly for one column
            template<typename D>
            inline void operator=(const Eigen::MatrixBase<D> &in) {
                m_velocity.noalias()= in;
                cache= false;
            }

            template<typename D>
            inline void operator-=(const Eigen::MatrixBase<D> &in) {
                m_velocity.noalias()-= in;
                cache= false;
            }
            template<typename D>
            inline void operator+=(const Eigen::MatrixBase<D> &in) {
                m_velocity.noalias()+= in;
                cache= false;
            }
            template<typename D>
            inline void operator*=(const Eigen::MatrixBase<D> &in) {
                m_velocity.noalias()*= in;
                cache= false;
            }
        };
    } // namespace Math
} // namespace ARDL

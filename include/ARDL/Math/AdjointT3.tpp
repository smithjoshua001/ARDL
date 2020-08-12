#pragma once
#include <ostream>

#include "ARDL/Util/Math.hpp"

namespace ARDL {
    using namespace Util::Math;
    namespace Math {
        /**
         * @brief Adjoint Group matrix (Transformation in Group group)
         *
         * @tparam Group
         */
        template<typename D, typename Group>
        class AdjointT
#ifdef ARDL_EXP_TEM
            : LieExpression<AdjointT<Group>>
#endif
        {
             private:
            constexpr Group &derived() { return static_cast<Group &>(*this); }
            Eigen::Matrix<D, 6, 6> m_Matrix;

            void calcMatrix()= delete;
            template<typename Derived>
            void calcMatrix(const Eigen::MatrixBase<Derived> &out) {
                derived().calcMatrix();
            };

             private:
            // temporary variables
            Eigen::Matrix<D, 3, 3> t_P_skew, rTh_skew, rThmp_skew, tmp_inertia;

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            AdjointT(){};

            AdjointT<D, Group> &operator=(const AdjointT<D, Group> &in) {
                derived().operator=(in);
                return *this;
            }

            void setIdentity() { derived().setIdentity(); }

            Eigen::Matrix<D, 6, 6> const &getMatrix(bool recalc= true) {
                if(recalc) { calcMatrix(); }
                return m_Matrix;
            }

            Eigen::Matrix<D, 3, 3> &getRRef() { return m_R; }
            Eigen::Matrix<D, 3, 1> &getPRef() { return m_P; }

            void applyTo(const AdjointT<D, Group> &trans,
                         AdjointT<D, Group> &out) const {
                out.m_P= m_P + this->m_R * trans.m_P;
                out.m_R= this->m_R * trans.m_R;
            }

            void applyRotationTo(const AdjointT<D, Group> &trans,
                                 AdjointT<D, Group> &out) {
                out.m_P= this->m_R * trans.m_P;
                out.m_R= this->m_R * trans.m_R;
            }

            void apply(const AdjointT<D, Group> &trans,
                       AdjointT<D, Group> &out) const {
                out.m_P= trans.m_R * m_P + trans.m_P;
                out.m_R= trans.m_R * this->m_R;
            }
            void applyRotation(const AdjointT<D, Group> &trans,
                               AdjointT<D, Group> &out) {
                out.m_P= trans.m_P;
                out.m_R= trans.m_R * this->m_R;
            }

            void applyTo(const AdjointT<D, Group> &trans) {
                m_P.noalias()+= m_R * trans.m_P;
                m_R*= trans.m_R;
            }

            void apply(const AdjointT<D, Group> &trans) {
                m_P= trans.m_R * m_P + trans.m_P;
                m_R= trans.m_R * this->m_R;
            }

            void inverse() {
                m_R.transposeInPlace();
                m_P= -m_R * m_P;
            }

            void applyInverseTo(const AdjointT<D, Group> &trans,
                                AdjointT<D, Group> &out) const {
                // out.m_P = trans.R.transpose() * (m_P - trans.m_P);
                // out.R = trans.R.transpose() * R;
                out.m_P= m_R.transpose() * (trans.m_P - m_P);

                out.m_R= this->m_R.transpose() * trans.m_R;
            }
            void applyInverse(const AdjointT<D, Group> &trans,
                              AdjointT<D, Group> &out) const {
                out.m_P= trans.m_R * m_R.transpose() * -m_P + trans.m_P;
                out.m_R= trans.m_R * this->m_R.transpose();
                // out.m_P = (-R * m_P) + this->R.transpose() * trans.m_P;
                // out.R = trans.R * this->R.transpose();
                // out.m_P = R.transpose() * (trans.m_P - m_P);
                // out.R = this->R.transpose() * trans.R;
            }

            /**
             * @brief Apply adjoint transform ([linear, angular])
             *
             * @param vector Vector to transform
             * @param out Output Vector
             */
            template<typename Derived2, typename Derived>
            void apply(Eigen::MatrixBase<Derived2> const &vector,
                       Eigen::MatrixBase<Derived> const &out) const {
                // LOG_DEBUG_LEVEL6("{} \n----------\n {} \n---------\n {}",
                // vector.transpose(), R, m_P); Eigen::Matrix<D, 3, 3> skew;
                // skewMatrix(m_P, skew); const_cast<Eigen::MatrixBase<Derived>
                // &>(out).template tail<3>().noalias() = m_R * vector.template
                // tail<3>(); const_cast<Eigen::MatrixBase<Derived>
                // &>(out).template head<3>().noalias() = m_P.cross(out.template
                // tail<3>()); const_cast<Eigen::MatrixBase<Derived>
                // &>(out).template head<3>().noalias() += m_R * vector.template
                // head<3>();

                mt_apply.template tail<3>().noalias()=
                    m_R * vector.template tail<3>();
                mt_apply.template head<3>().noalias()=
                    m_R * vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template head<3>()
                    .noalias()= m_P.cross(mt_apply.template tail<3>());
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template head<3>()
                    .noalias()+= mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template tail<3>()= mt_apply.template tail<3>();

                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // tail<3>() = m_R * vector.template tail<3>();
                // const_cast<Eigen::MatrixBase<Derived>
                // &>(out).template head<3>() = m_R * (vector.template
                // head<3>()) + m_P.cross(m_R * vector.template tail<3>());
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // tail<3>() = m_R * vector.template tail<3>();
            }

            template<typename Derived2, typename Derived>
            void applyInverse(const Eigen::MatrixBase<Derived2> &vector,
                              const Eigen::MatrixBase<Derived> &out) const {
                // Eigen::Matrix<D, 3, 3> skew;
                // skewMatrix(m_P, skew);
                mt_apply.template tail<3>().noalias()=
                    m_P.cross(vector.template tail<3>());
                mt_apply.template head<3>().noalias()=
                    m_R.transpose() * vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template head<3>()
                    .noalias()= mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template head<3>()
                    .noalias()-=
                    (m_R.transpose() * mt_apply.template tail<3>());
                mt_apply.template tail<3>().noalias()=
                    m_R.transpose() * vector.template tail<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template tail<3>()= mt_apply.template tail<3>();
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // tail<3>() = m_R.transpose() * vector.template tail<3>();
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // head<3>() = m_R.transpose() * (vector.template head<3>()) -
                // m_R.transpose() * m_P.cross(vector.template tail<3>());
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // tail<3>() = m_R.transpose() * vector.template tail<3>();
            }

            template<typename Derived3, typename Derived2, typename Derived>
            void
            applyInverseMat(const Eigen::MatrixBase<Derived2> &input,
                            const Eigen::MatrixBase<Derived> &out,
                            const Eigen::MatrixBase<Derived3> &buffer) const {
                skewMatrix(m_P, mt_skew);
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // block(0, 0, 3, input.cols()) = m_R.transpose() *
                // (input.template block(0, 0, 3, input.cols()) - mt_skew *
                // input.template block(3, 0, 3, input.cols()));
                // const_cast<Eigen::MatrixBase<Derived> &>(out).template
                // block(3, 0, 3, input.cols()) = m_R.transpose() *
                // input.template block(3, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block(3, 0, 3, input.cols())
                    .noalias()=
                    mt_skew * input.template block(3, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block(0, 0, 3, input.cols())
                    .noalias()= m_R.transpose() *
                                input.template block(0, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block(0, 0, 3, input.cols())
                    .noalias()= buffer.template block(0, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block(0, 0, 3, input.cols())
                    .noalias()-= m_R.transpose() *
                                 buffer.template block(3, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block(3, 0, 3, input.cols())
                    .noalias()= m_R.transpose() *
                                input.template block(3, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block(3, 0, 3, input.cols())
                    .noalias()= buffer.template block(3, 0, 3, input.cols());
            }

            template<typename Derived>
            void applyXIX(const Eigen::Matrix<D, 10, 1> &inertialParams,
                          Eigen::MatrixBase<Derived> const &out) {
                const_cast<Eigen::MatrixBase<Derived> &>(out)= inertialParams;
                skewMatrix(m_P, t_P_skew);
                t_P_skew= t_P_skew * m_R;
                skewMatrix(out.template block<3, 1>(1, 0), rTh_skew);
                //  skewMatrix(R * out.template block<3, 1>(1, 0) +
                //  inertialParams(0) * m_P, rThmp_skew);
                tmp_inertia.setZero();
                tmp_inertia(0, 0)= inertialParams(4);
                tmp_inertia(1, 1)= inertialParams(5);
                tmp_inertia(2, 2)= inertialParams(6);
                tmp_inertia(0, 1)= tmp_inertia(1, 0)= inertialParams(7);
                tmp_inertia(0, 2)= tmp_inertia(2, 0)= inertialParams(8);
                tmp_inertia(1, 2)= tmp_inertia(2, 1)= inertialParams(9);
                tmp_inertia=
                    m_R.transpose() * tmp_inertia * m_R +
                    inertialParams(0) * t_P_skew.transpose() * t_P_skew -
                    t_P_skew.transpose() * rTh_skew * m_R +
                    m_R.transpose() * rTh_skew * t_P_skew;

                const_cast<Eigen::MatrixBase<Derived> &>(out)(4)=
                    tmp_inertia(0, 0);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(5)=
                    tmp_inertia(1, 1);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(6)=
                    tmp_inertia(2, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(7)=
                    tmp_inertia(0, 1);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(8)=
                    tmp_inertia(0, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(9)=
                    tmp_inertia(1, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block<3, 1>(1, 0)=
                    m_R.transpose() *
                        inertialParams.template block<3, 1>(1, 0) -
                    inertialParams(0) * m_R.transpose() * m_P;
            }

            template<typename Derived>
            void applyInverseXIX(const Eigen::Matrix<D, 10, 1> &inertialParams,
                                 Eigen::MatrixBase<Derived> const &out) {
                const_cast<Eigen::MatrixBase<Derived> &>(out)= inertialParams;

                skewMatrix(-m_P, t_P_skew);
                skewMatrix(m_R * out.template block<3, 1>(1, 0), rTh_skew);
                skewMatrix(m_R * out.template block<3, 1>(1, 0) +
                               inertialParams(0) * m_P,
                           rThmp_skew);
                tmp_inertia.setZero();
                tmp_inertia(0, 0)= inertialParams(4);
                tmp_inertia(1, 1)= inertialParams(5);
                tmp_inertia(2, 2)= inertialParams(6);
                tmp_inertia(0, 1)= tmp_inertia(1, 0)= inertialParams(7);
                tmp_inertia(0, 2)= tmp_inertia(2, 0)= inertialParams(8);
                tmp_inertia(1, 2)= tmp_inertia(2, 1)= inertialParams(9);
                tmp_inertia= m_R * tmp_inertia * m_R.transpose() -
                             t_P_skew.transpose() * rThmp_skew +
                             rTh_skew * t_P_skew;

                const_cast<Eigen::MatrixBase<Derived> &>(out)(4)=
                    tmp_inertia(0, 0);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(5)=
                    tmp_inertia(1, 1);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(6)=
                    tmp_inertia(2, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(7)=
                    tmp_inertia(0, 1);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(8)=
                    tmp_inertia(0, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)(9)=
                    tmp_inertia(1, 2);
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block<3, 1>(1, 0)=
                    m_R * inertialParams.template block<3, 1>(1, 0) +
                    inertialParams(0) * m_P;
            }

            friend std ::ostream &operator<<(std::ostream &out,
                                             const AdjointT<D, Group> &At) {
                return out << "R:\n" << At.m_R << "\np:\n" << At.m_P;
            }
#ifdef ARDL_EXP_TEM
            template<typename Exp2>
            void operator=(LieExpression<Exp2> &in) {
                in(*this);
            }
#endif
        };
    } // namespace Math
} // namespace ARDL

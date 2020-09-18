#pragma once
#include <iostream>

#include "ARDL/Util/Math.hpp"

// #ifdef ARDL_EXP_TEM
//             : LieExpression<AdjointSE3<T>>
// #endif
namespace ARDL {
    using namespace Util::Math;
    namespace Math {
        /**
         * @brief Adjoint SE3 matrix (Transformation in SE3 Lie group)
         *
         * @tparam T
         */
        template<typename T>
        class AdjointSE3
        {
             private:
            Eigen::Matrix<T, 3, 3> m_R;
            Eigen::Matrix<T, 3, 1> m_P;
            Eigen::Matrix<T, 6, 6> m_Matrix;
            mutable Eigen::Matrix<T, 6, 1> mt_apply;

            mutable Eigen::Matrix<T, 3, 3> mt_skew, mt_R;

            /**
             * @brief calcMatrix Compute the Adjoint Matrix
             *
             */
            void calcMatrix() {
                m_Matrix.setZero();
                m_Matrix.template block<3, 3>(0, 0)= m_R;
                m_Matrix.template block<3, 3>(3, 3)= m_R;
                ARDL::Util::Math::skewMatrix(m_P, m_Matrix.template block<3, 3>(0, 3));
                m_Matrix.template block<3, 3>(0, 3)= m_Matrix.template block<3, 3>(0, 3) * m_R;
            }

             private:
            // temporary variables
            Eigen::Matrix<T, 3, 3> t_P_skew, rTh_skew, rThmp_skew, tmp_inertia;

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            template<typename DerivedR, typename DerivedP>
            AdjointSE3(const Eigen::MatrixBase<DerivedR> &R, const Eigen::MatrixBase<DerivedP> &m_P) {
                this->m_R= R;
                this->m_P= m_P;
                mt_skew.setZero();
                mt_R.setZero();
                // calcMatrix();
            }

            constexpr AdjointSE3() {
                this->setIdentity();
                mt_skew.setZero();
                mt_R.setZero();
            }

            AdjointSE3<T> &operator=(const AdjointSE3<T> &in) {
                this->m_R= in.m_R;
                this->m_P= in.m_P;
                mt_skew.setZero();
                mt_R.setZero();
                return *this;
            }

            void setIdentity() {
                m_R= Eigen::Matrix<T, 3, 3>::Identity();
                m_P= Eigen::Matrix<T, 3, 1>::Zero();
                mt_skew.setZero();
                mt_R.setZero();
                // calcMatrix();
            }

            template<typename Derived>
            void setR(const Eigen::MatrixBase<Derived> &R) {
                this->m_R= R;
            }

            Eigen::Matrix<T, 3, 3> const &getR() const { return m_R; }

            Eigen::Matrix<T, 3, 1> const &getP() const { return m_P; }

            Eigen::Matrix<T, 6, 6> const &getMatrix(bool recalc= true) {
                if(recalc) { calcMatrix(); }
                return m_Matrix;
            }
            Eigen::Matrix<T, 6, 6> &getMatrixRef(bool recalc= true) {
                if(recalc) { calcMatrix(); }
                return m_Matrix;
            }

            Eigen::Matrix<T, 3, 3> &getRRef() { return m_R; }
            Eigen::Matrix<T, 3, 1> &getPRef() { return m_P; }

            // X X_{trans}
            void applyTo(const AdjointSE3<T> &trans, AdjointSE3<T> &out) const {
                out.m_P= m_P + this->m_R * trans.m_P;
                out.m_R= this->m_R * trans.m_R;
            }
            // X_{trans} X
            void applyRotationTo(const AdjointSE3<T> &trans, AdjointSE3<T> &out) {
                out.m_P= this->m_R * trans.m_P;
                out.m_R= this->m_R * trans.m_R;
            }

            void apply(const AdjointSE3<T> &trans, AdjointSE3<T> &out) const {
                out.m_P= trans.m_R * m_P + trans.m_P;
                out.m_R= trans.m_R * this->m_R;
            }
            void applyRotation(const AdjointSE3<T> &trans, AdjointSE3<T> &out) {
                out.m_P= trans.m_P;
                out.m_R= trans.m_R * this->m_R;
            }

            void applyTo(const AdjointSE3<T> &trans) {
                m_P.noalias()+= m_R * trans.m_P;
                m_R*= trans.m_R;
            }

            void apply(const AdjointSE3<T> &trans) {
                m_P= trans.m_R * m_P + trans.m_P;
                m_R= trans.m_R * this->m_R;
            }

            void inverse() {
                m_R.transposeInPlace();
                m_P= -m_R * m_P;
            }

            // //rightMult
            void applyInverseTo(const AdjointSE3<T> &trans, AdjointSE3<T> &out) const {
                mt_apply.template head<3>().noalias()= m_R.transpose() * m_P;
                mt_apply.template tail<3>().noalias()= m_R.transpose() * trans.m_P;
                out.m_P.noalias()= mt_apply.template tail<3>() - mt_apply.template head<3>();
                mt_R.noalias()= m_R.transpose() * trans.m_R;
                out.m_R= mt_R;
            }

            // //leftMult
            void applyInverse(const AdjointSE3<T> &trans, AdjointSE3<T> &out) const {
                // R^T p
                mt_apply.template head<3>().noalias()= m_R.transpose() * m_P;
                // R' R^T p
                mt_apply.template tail<3>().noalias()= trans.m_R * mt_apply.template head<3>();
                // p'
                mt_apply.template head<3>().noalias()= trans.m_P;
                // p' - R' R^T p
                mt_apply.template head<3>().noalias()-= mt_apply.template tail<3>();

                out.m_P.noalias()= mt_apply.template head<3>();

                mt_R.noalias()= trans.m_R * this->m_R.transpose();

                out.m_R= mt_R;
            }

            /**
             * @brief Apply adjoint transform ([linear, angular])
             *
             * @param vector Vector to transform
             * @param out Output Vector
             */
            template<typename Derived2, typename Derived,
                     typename std::enable_if<Derived2::ColsAtCompileTime == 1, int>::type= 0>
            void apply(Eigen::MatrixBase<Derived2> const &vector, Eigen::MatrixBase<Derived> const &out) const {
                // if constexpr(Derived2::ColsAtCompileTime==1){
                /*if constexpr(Derived2::ColsAtCompileTime!=1){
                std::cout<<"SIZE : "<<Derived2::BlockCols<<std::endl;
                }*/
                mt_apply.template tail<3>().noalias()= m_R * vector.template tail<3>();
                mt_apply.template head<3>().noalias()= m_R * vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()=
                    m_P.cross(mt_apply.template tail<3>());
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()+=
                    mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template tail<3>().noalias()= mt_apply.template tail<3>();
                //  }
            }

            template<typename Derived2, typename Derived,
                     typename std::enable_if<Derived2::ColsAtCompileTime != 1, int>::type= 0>
            void apply(Eigen::MatrixBase<Derived2> const &vector, Eigen::MatrixBase<Derived> const &out) const {
                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(0, 0, 3, vector.cols()).noalias()=
                    m_R * vector.template block<3, -1>(0, 0, 3, vector.cols());
                skewMatrix(m_P, mt_skew);

                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(3, 0, 3, vector.cols()).noalias()=
                    m_R * vector.template block<3, -1>(3, 0, 3, vector.cols());

                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(0, 0, 3, vector.cols()).noalias()+=
                    mt_skew * out.template block<3, -1>(3, 0, 3, vector.cols());
            }

            template<typename Derived2, typename Derived>
            void applyRotation(Eigen::MatrixBase<Derived2> const &vector, Eigen::MatrixBase<Derived> const &out) const {
                mt_apply.template tail<3>().noalias()= m_R * vector.template tail<3>();
                mt_apply.template head<3>().noalias()= m_R * vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()= mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template tail<3>().noalias()= mt_apply.template tail<3>();
            }

            template<typename Derived2, typename Derived>
            void applyInverse(const Eigen::MatrixBase<Derived2> &vector, const Eigen::MatrixBase<Derived> &out) const {
                mt_apply.template tail<3>().noalias()= m_P.cross(vector.template tail<3>());
                mt_apply.template head<3>().noalias()= m_R.transpose() * vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()= mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()-=
                    (m_R.transpose() * mt_apply.template tail<3>());
                mt_apply.template tail<3>().noalias()= m_R.transpose() * vector.template tail<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template tail<3>().noalias()= mt_apply.template tail<3>();
            }

            template<typename Derived2, typename Derived>
            void applyInverseTranslation(const Eigen::MatrixBase<Derived2> &vector,
                                         const Eigen::MatrixBase<Derived> &out) const {
                mt_apply.template head<3>().noalias()= vector.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()= mt_apply.template head<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template head<3>().noalias()-=
                    m_P.cross(vector.template tail<3>());
                mt_apply.template tail<3>().noalias()= vector.template tail<3>();
                const_cast<Eigen::MatrixBase<Derived> &>(out).template tail<3>()= mt_apply.template tail<3>();
            }

            template<typename Derived3, typename Derived2, typename Derived>
            void applyInverseMat(const Eigen::MatrixBase<Derived2> &input, const Eigen::MatrixBase<Derived> &out,
                                 const Eigen::MatrixBase<Derived3> &buffer) const {
                skewMatrix(m_P, mt_skew);
                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, -1>(3, 0, 3, input.cols())
                    .noalias()= mt_skew * input.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, -1>(0, 0, 3, input.cols())
                    .noalias()= m_R.transpose() * input.template block<3, Eigen::Dynamic>(0, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(0, 0, 3, input.cols()).noalias()=
                    buffer.template block<3, Eigen::Dynamic>(0, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(0, 0, 3, input.cols()).noalias()-=
                    m_R.transpose() * buffer.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived3> &>(buffer)
                    .template block<3, -1>(3, 0, 3, input.cols())
                    .noalias()= m_R.transpose() * input.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());
                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(3, 0, 3, input.cols()).noalias()=
                    buffer.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());
            }

            template<typename Derived2, typename Derived>
            void applyInverseMat(const Eigen::MatrixBase<Derived2> &input, const Eigen::MatrixBase<Derived> &out) const {
                skewMatrix(m_P, mt_skew);
                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block<3, -1>(3, 0, 3, input.cols())
                    .noalias()= mt_skew * input.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block<3, -1>(0, 0, 3, input.cols())
                    .noalias()= m_R.transpose() * input.template block<3, Eigen::Dynamic>(0, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived> &>(out).template block<3, -1>(0, 0, 3, input.cols()).noalias()-=
                    m_R.transpose() * out.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());

                const_cast<Eigen::MatrixBase<Derived> &>(out)
                    .template block<3, -1>(3, 0, 3, input.cols())
                    .noalias()= m_R.transpose() * input.template block<3, Eigen::Dynamic>(3, 0, 3, input.cols());
            }

            /**
             *  @brief
             *
             *  @param out
             *  @param At
             **/
            friend std ::ostream &operator<<(std::ostream &out, const AdjointSE3<T> &At) {
                return out << "R:\n" << At.m_R << "\np:\n" << At.m_P;
            }
// #ifdef ARDL_EXP_TEM
//             /**
//              *  @brief
//              *
//              *  @param in
//              **/
//             template<typename Exp2>
//             void operator=(LieExpression<Exp2> &in) {
//                 in(*this);
//             }
// #endif
        };
    } // namespace Math
} // namespace ARDL

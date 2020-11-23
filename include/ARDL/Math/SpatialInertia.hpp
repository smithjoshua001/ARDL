#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ARDL/typedefs.hpp"
#include "ARDL/Util/Math.hpp"
#include "ARDL/Math/Pose.hpp"
// #include "ARDL/Math/Motion.hpp"
using namespace std;
using namespace Eigen;
namespace ARDL {
    namespace Math {
        template<typename T>
        class SpatialInertia {
             private:
            Matrix<T, 6, 6> m_spatialInertia;
            Matrix<T, 3, 3> inertia;
            T mass;
            Matrix<T, 3, 1> momentMass;

            Matrix<T, 3, 3> mt_skewP;
            Matrix<T, 3, 3> mt_skewR;

             public:
            T &getMass() { return mass; }

            Matrix<T, 3, 1> &getMomentMass() { return momentMass; }

            Matrix<T, 3, 3> &getInertia() { return inertia; }

            const Matrix<T, 6, 6> &getSpatialInertia() { return m_spatialInertia; }

            constexpr SpatialInertia() {
                m_spatialInertia.setZero();
                inertia.setIdentity();
                mass= 0;
                momentMass.setZero();
            }
            const Matrix<T, 6, 6> &calculateSpatialInertia() {
                skewMatrix(momentMass, m_spatialInertia.template block<3, 3>(3, 0));
                m_spatialInertia.template block<3, 3>(0, 3)= -m_spatialInertia.template block<3, 3>(3, 0);
                m_spatialInertia.template block<3, 3>(3, 3)= inertia;

                m_spatialInertia.template block<3, 3>(0, 0)= mass * Matrix<T, 3, 3>::Identity();
                return m_spatialInertia;
            }
            template<typename Derived>
            SpatialInertia(const MatrixBase<Derived> &in) {
                m_spatialInertia.setZero();
                mass= in(0);
                momentMass= in.template segment<3>(1);
                inertia.diagonal()= in.template segment<3>(4);
                inertia(0, 1)= inertia(1, 0)= in(7);
                inertia(0, 2)= inertia(2, 0)= in(8);
                inertia(1, 2)= inertia(2, 1)= in(9);
            }
            SpatialInertia(const SpatialInertia &copy) {
                m_spatialInertia.setZero();
                this->inertia= copy.inertia;
                this->mass= copy.mass;
                this->momentMass= copy.momentMass;
            }
            SpatialInertia<T> &operator=(const SpatialInertia<T> &other) {
                m_spatialInertia.setZero();
                this->inertia= other.inertia;
                this->mass= other.mass;
                this->momentMass= other.momentMass;
                return *this;
            }
            SpatialInertia<T> &operator+=(const SpatialInertia<T> &rhs) {
                mass+= rhs.mass;
                momentMass+= rhs.momentMass;
                inertia+= rhs.inertia;
                return *this;
            }
            template<typename D>
            SpatialInertia<T> &operator=(const MatrixBase<D> &in) {
                m_spatialInertia.setZero();
                mass= in(0);
                momentMass= in.template segment<3>(1);
                inertia.diagonal()= in.template segment<3>(4);
                inertia(0, 1)= inertia(1, 0)= in(7);
                inertia(0, 2)= inertia(2, 0)= in(8);
                inertia(1, 2)= inertia(2, 1)= in(9);
                return *this;
            }
            template<typename D>
            SpatialInertia<T> &operator+=(const MatrixBase<D> &in) {
                mass+= in(0);
                momentMass+= in.template segment<3>(1);
                inertia.diagonal()+= in.template segment<3>(4);
                inertia(0, 1)= inertia(1, 0)+= in(7);
                inertia(0, 2)= inertia(2, 0)+= in(8);
                inertia(1, 2)= inertia(2, 1)+= in(9);
                return *this;
            }

            template<typename D>
            void toVector(const Eigen::MatrixBase<D> &out) {
                const_cast<Eigen::MatrixBase<D> &>(out)(0)= mass;
                const_cast<Eigen::MatrixBase<D> &>(out).template segment<3>(1)= momentMass;
                const_cast<Eigen::MatrixBase<D> &>(out).template segment<3>(4)= inertia.diagonal();
                const_cast<Eigen::MatrixBase<D> &>(out)(7)= inertia(0, 1);
                const_cast<Eigen::MatrixBase<D> &>(out)(8)= inertia(0, 2);
                const_cast<Eigen::MatrixBase<D> &>(out)(9)= inertia(1, 2);
            }
            void applyXIX(const Pose<T> &trans) {
                // h
                mt_skewR.col(0)= momentMass;
                // R^T h
                momentMass.noalias()= trans.getR().transpose() * mt_skewR.col(0);

                // R^T * I
                mt_skewR.noalias()= trans.getR().transpose() * inertia.template selfadjointView<Eigen::Upper>();

                // R^T I R
                inertia.noalias()= mt_skewR * trans.getR();

                //R^T p
                mt_skewR.col(0).noalias()= trans.getR().transpose() * trans.getP();
                // S(R^T p)
                skewMatrix(mt_skewR.col(0), mt_skewP);
                // S(R^T h)
                skewMatrix(momentMass, mt_skewR);
                // R^T I R + S(R^T p)S(R^T h)
                inertia.noalias()+= mt_skewP * mt_skewR;


                // R^T h - m R^{T} p
                momentMass.noalias()-= mass * trans.getR().transpose() *trans.getP();
                
                // S(R^T h - m R^T p)
                skewMatrix(momentMass, mt_skewR);
                //  R^T I R + S(R^T p)S(R^T h) +  S(R^T h - m R^T p)S(R^T p)
                inertia.noalias()+= mt_skewR * mt_skewP;

            }
            void applyInverseXIX(const Pose<T> &trans) {
                // R = RT
                // p = - RT p
                // h
                mt_skewR.col(0)= momentMass;
                // R h
                momentMass.noalias()= trans.getR() * mt_skewR.col(0);

                // R * I
                mt_skewR.noalias()= trans.getR() * inertia.template selfadjointView<Eigen::Upper>();

                // R I R^T
                inertia.noalias()= mt_skewR * trans.getR().transpose();

                //- R R^T p
                // mt_skewR.col(0).noalias()= -trans.getP();
                // S(- R R^T p)
                skewMatrix(trans.getP(), mt_skewP);
                // S(R h)
                skewMatrix(momentMass, mt_skewR);
                // R I R^T + S(- R R^T p)S(R h)
                inertia.noalias()-= mt_skewP * mt_skewR;

                // mt_skewR.col(0).noalias()=trans.getP();
                // R h + m R R^T p
                momentMass.noalias()+= mass  *trans.getP();
                
                // S(R h + m R R^T p)
                skewMatrix(momentMass, mt_skewR);
                //  R I R^T + S(- R R p)S(R h) +  S(R h + m R R^T p)S(- R R^T p)
                inertia.noalias()-= mt_skewR * mt_skewP;
            }


            template<typename inD, typename bufD>
            void apply(const MatrixBase<inD> &in, const MatrixBase<bufD> &buffer) {
                const_cast<MatrixBase<bufD> &>(buffer).template block<3, Eigen::Dynamic>(0, 0, 3, in.cols()).noalias()=
                    mass * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
                mt_skewP.setZero();
                skewMatrix(momentMass, mt_skewP);
                const_cast<MatrixBase<bufD> &>(buffer).template block<3, Eigen::Dynamic>(0, 0, 3, in.cols()).noalias()-=
                    mt_skewP * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<MatrixBase<bufD> &>(buffer).template block<3, Eigen::Dynamic>(3, 0, 3, in.cols()).noalias()=
                    inertia * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<MatrixBase<bufD> &>(buffer).template block<3, Eigen::Dynamic>(3, 0, 3, in.cols()).noalias()+=
                    mt_skewP * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
                const_cast<MatrixBase<inD> &>(in)= buffer;
            }

            template<typename inD, typename bufD>
            void applyOut(const MatrixBase<inD> &in, const MatrixBase<bufD> &out) {
                const_cast<MatrixBase<bufD> &>(out).template block<3, Eigen::Dynamic>(0, 0, 3, in.cols()).noalias()=
                    mass * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
                mt_skewP.setZero();
                skewMatrix(momentMass, mt_skewP);
                const_cast<MatrixBase<bufD> &>(out).template block<3, Eigen::Dynamic>(0, 0, 3, in.cols()).noalias()-=
                    mt_skewP * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<MatrixBase<bufD> &>(out).template block<3, Eigen::Dynamic>(3, 0, 3, in.cols()).noalias()=
                    inertia * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
                const_cast<MatrixBase<bufD> &>(out).template block<3, Eigen::Dynamic>(3, 0, 3, in.cols()).noalias()+=
                    mt_skewP * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
            }
        };
    } // namespace Math
} // namespace ARDL

#pragma once

#include "ARDL/Math/Pose.hpp"
#include "ARDL/Util/Math.hpp"
#include "ARDL/typedefs.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include "ARDL/Math/Motion.hpp"
using namespace std;
using namespace Eigen;
namespace ARDL::Math {
template <typename T> class SpatialInertia {
  private:
    Matrix<T, 6, 6> m_spatialInertia;
    Matrix<T, 3, 3> m_inertia;
    T m_mass;
    Matrix<T, 3, 1> m_momentMass;

    Matrix<T, 3, 3> m_skewP;
    Matrix<T, 3, 3> m_skewR;

  public:
    T &getMass() { return m_mass; }

    Matrix<T, 3, 1> &getMomentMass() { return m_momentMass; }

    Matrix<T, 3, 3> &getInertia() { return m_inertia; }

    const Matrix<T, 6, 6> &getSpatialInertia() { return m_spatialInertia; }

    ~SpatialInertia() = default;

    constexpr SpatialInertia() {
        m_spatialInertia.setZero();
        m_inertia.setIdentity();
        m_mass = 0;
        m_momentMass.setZero();
    }

    const Matrix<T, 6, 6> &calculateSpatialInertia() {
        skewMatrix(m_momentMass, m_spatialInertia.template block<3, 3>(3, 0));
        m_spatialInertia.template block<3, 3>(0, 3) =
            -m_spatialInertia.template block<3, 3>(3, 0);
        m_spatialInertia.template block<3, 3>(3, 3) = m_inertia;

        m_spatialInertia.template block<3, 3>(0, 0) =
            m_mass * Matrix<T, 3, 3>::Identity();
        return m_spatialInertia;
    }
    template <typename Derived>
    explicit SpatialInertia(const MatrixBase<Derived> &in) {
        m_spatialInertia.setZero();
        m_mass = in(0);
        m_momentMass = in.template segment<3>(1);
        m_inertia.diagonal() = in.template segment<3>(4);

        m_inertia(0, 1) = m_inertia(1, 0) = in(7);

        m_inertia(0, 2) = m_inertia(2, 0) = in(8);

        m_inertia(1, 2) = m_inertia(2, 1) = in(9);
    }
    SpatialInertia(const SpatialInertia &copy) {
        m_spatialInertia.setZero();
        this->m_inertia = copy.m_inertia;
        this->m_mass = copy.m_mass;
        this->m_momentMass = copy.m_momentMass;
    }
    SpatialInertia<T> &operator=(const SpatialInertia<T> &other) {
        if (this == &other) {
            return *this;
        }
        m_spatialInertia.setZero();
        this->m_inertia = other.m_inertia;
        this->m_mass = other.m_mass;
        this->m_momentMass = other.m_momentMass;
        return *this;
    }
    SpatialInertia(SpatialInertia &&copy) noexcept {
        m_spatialInertia.setZero();
        this->m_inertia = copy.m_inertia;
        this->m_mass = copy.m_mass;
        this->m_momentMass = copy.m_momentMass;
    }
    SpatialInertia<T> &operator=(SpatialInertia<T> &&other) noexcept {
        if (this == &other) {
            return *this;
        }
        m_spatialInertia.setZero();
        this->m_inertia = other.m_inertia;
        this->m_mass = other.m_mass;
        this->m_momentMass = other.m_momentMass;
        return *this;
    }
    SpatialInertia<T> &operator+=(const SpatialInertia<T> &rhs) {
        m_mass += rhs.m_mass;
        m_momentMass += rhs.m_momentMass;
        m_inertia += rhs.m_inertia;
        return *this;
    }
    template <typename D>
    SpatialInertia<T> &operator=(const MatrixBase<D> &in) {
        m_spatialInertia.setZero();
        m_mass = in(0);
        m_momentMass = in.template segment<3>(1);
        m_inertia.diagonal() = in.template segment<3>(4);

        m_inertia(0, 1) = m_inertia(1, 0) = in(7);

        m_inertia(0, 2) = m_inertia(2, 0) = in(8);

        m_inertia(1, 2) = m_inertia(2, 1) = in(9);
        return *this;
    }
    template <typename D>
    SpatialInertia<T> &operator+=(const MatrixBase<D> &in) {
        m_mass += in(0);
        m_momentMass += in.template segment<3>(1);
        m_inertia.diagonal() += in.template segment<3>(4);

        m_inertia(0, 1) = m_inertia(1, 0) += in(7);

        m_inertia(0, 2) = m_inertia(2, 0) += in(8);

        m_inertia(1, 2) = m_inertia(2, 1) += in(9);
        return *this;
    }

    template <typename D> void toVector(const Eigen::MatrixBase<D> &out) {
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out)(0) = m_mass;
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out).template segment<3>(1) =
            m_momentMass;
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out).template segment<3>(4) =
            m_inertia.diagonal();
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out)(7) = m_inertia(0, 1);
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out)(8) = m_inertia(0, 2);
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<Eigen::MatrixBase<D> &>(out)(9) = m_inertia(1, 2);
    }
    void applyXIX(const Pose<T> &trans) {
        // h
        m_skewR.col(0) = m_momentMass;
        // R^T h
        m_momentMass.noalias() = trans.getR().transpose() * m_skewR.col(0);

        // R^T * I
        m_skewR.noalias() = trans.getR().transpose() *
                            m_inertia.template selfadjointView<Eigen::Upper>();

        // R^T I R
        m_inertia.noalias() = m_skewR * trans.getR();

        // R^T p
        m_skewR.col(0).noalias() = trans.getR().transpose() * trans.getP();
        // S(R^T p)
        skewMatrix(m_skewR.col(0), m_skewP);
        // S(R^T h)
        skewMatrix(m_momentMass, m_skewR);
        // R^T I R + S(R^T p)S(R^T h)
        m_inertia.noalias() += m_skewP * m_skewR;

        // R^T h - m R^{T} p
        m_momentMass.noalias() -=
            m_mass * trans.getR().transpose() * trans.getP();

        // S(R^T h - m R^T p)
        skewMatrix(m_momentMass, m_skewR);
        //  R^T I R + S(R^T p)S(R^T h) +  S(R^T h - m R^T p)S(R^T p)
        m_inertia.noalias() += m_skewR * m_skewP;
    }
    void applyInverseXIX(const Pose<T> &trans) {
        // R = RT
        // p = - RT p
        // h
        m_skewR.col(0) = m_momentMass;
        // R h
        m_momentMass.noalias() = trans.getR() * m_skewR.col(0);

        // R * I
        m_skewR.noalias() =
            trans.getR() * m_inertia.template selfadjointView<Eigen::Upper>();

        // R I R^T
        m_inertia.noalias() = m_skewR * trans.getR().transpose();

        //- R R^T p
        // mt_skewR.col(0).noalias()= -trans.getP();
        // S(- R R^T p)
        skewMatrix(trans.getP(), m_skewP);
        // S(R h)
        skewMatrix(m_momentMass, m_skewR);
        // R I R^T + S(- R R^T p)S(R h)
        m_inertia.noalias() -= m_skewP * m_skewR;

        // mt_skewR.col(0).noalias()=trans.getP();
        // R h + m R R^T p
        m_momentMass.noalias() += m_mass * trans.getP();

        // S(R h + m R R^T p)
        skewMatrix(m_momentMass, m_skewR);
        //  R I R^T + S(- R R p)S(R h) +  S(R h + m R R^T p)S(- R R^T p)
        m_inertia.noalias() -= m_skewR * m_skewP;
    }

    template <typename inD, typename bufD>
    void apply(const MatrixBase<inD> &in, const MatrixBase<bufD> &buffer) {
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(buffer)
            .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
            .noalias() =
            m_mass * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
        m_skewP.setZero();
        skewMatrix(m_momentMass, m_skewP);
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(buffer)
            .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
            .noalias() -=
            m_skewP * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(buffer)
            .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
            .noalias() = m_inertia * in.template block<3, Eigen::Dynamic>(
                                         3, 0, 3, in.cols());
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(buffer)
            .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
            .noalias() +=
            m_skewP * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<inD> &>(in) = buffer;
    }

    template <typename inD, typename bufD>
    void applyOut(const MatrixBase<inD> &in, const MatrixBase<bufD> &out) {
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(out)
            .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
            .noalias() =
            m_mass * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
        m_skewP.setZero();
        skewMatrix(m_momentMass, m_skewP);
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(out)
            .template block<3, Eigen::Dynamic>(0, 0, 3, in.cols())
            .noalias() -=
            m_skewP * in.template block<3, Eigen::Dynamic>(3, 0, 3, in.cols());
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(out)
            .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
            .noalias() = m_inertia * in.template block<3, Eigen::Dynamic>(
                                         3, 0, 3, in.cols());
        //NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<MatrixBase<bufD> &>(out)
            .template block<3, Eigen::Dynamic>(3, 0, 3, in.cols())
            .noalias() +=
            m_skewP * in.template block<3, Eigen::Dynamic>(0, 0, 3, in.cols());
    }
};
} // namespace ARDL::Math

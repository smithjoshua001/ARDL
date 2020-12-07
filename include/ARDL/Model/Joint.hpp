#pragma once
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <urdf_model/model.h>
#include <variant>

#include "ARDL/Math/Motion.hpp"
#include "ARDL/Math/Pose.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
template <typename T> struct Data;
}

using namespace ARDL::Math;
namespace ARDL::Model {
// Forward declare Link such that it can be used in a cyclic fashion.
template <typename T> class Link;

/**
         * @brief Base type for Joints
         *
         * @tparam T
         */
ARDL_NTYPE_CLASS_BASE(T, Joint) {
  protected:
    Eigen::Matrix<T, 6, 1> m_s;
    // Optimized Transform to origin
    Pose<T> m_optimOriginTransform;

    // transform to origin
    Pose<T> m_originTransform;
    // transform from origin to end of joint
    Pose<T> m_jointTransform;

    // Velocity of Joint
    Motion<T> m_adjPK;

    // Transformation of Joint
    Pose<T> m_adPK, m_adPKOptim;

    T m_staticFriction;
    T m_viscousFriction;

    VectorX<T> m_qMin, m_qMax, m_qdLimit, m_q, m_qd;

    Link<T> &m_parentLink;

    bool cache = false, qd_cache = false;

  public:

    ARDL_DERIVED(Joint<T>)
    Joint(const ARDL_BASE_TYPE(T, Joint) & copy, Link<T> & parentLink)
        : m_parentLink(parentLink) {
        m_s = copy.m_s;
        m_optimOriginTransform = copy.m_optimOriginTransform;
        m_originTransform = copy.m_originTransform;
        m_jointTransform = copy.m_jointTransform;
        m_adjPK = copy.m_adjPK;
        m_adPK = copy.m_adPK;
        m_adPKOptim = copy.m_adPKOptim;
        m_staticFriction = copy.m_staticFriction;
        m_viscousFriction = copy.m_viscousFriction;
        m_qMin = copy.m_qMin;
        m_qMax = copy.m_qMax;
        m_qdLimit = copy.m_qdLimit;
        m_q = copy.m_q;
        m_qd = copy.m_qd;
    }
    // Joint(const ARDL_BASE_TYPE(T, Joint) & copy)= delete;

    /****************** INITIALIZATION HANDLING **************/

    Joint(urdf::JointSharedPtr input, Link<T> & parentLink)
        : m_parentLink(parentLink) {
        m_originTransform.getPRef()
            << input->parent_to_joint_origin_transform.position.x,
            input->parent_to_joint_origin_transform.position.y,
            input->parent_to_joint_origin_transform.position.z;
        Eigen::Quaternion<double> t_quat;
        input->parent_to_joint_origin_transform.rotation.getQuaternion(
            t_quat.x(), t_quat.y(), t_quat.z(), t_quat.w());

        Eigen::Matrix<T, 3, 3> tempRotation =
            t_quat.toRotationMatrix().cast<T>();

        m_originTransform.setR(tempRotation);

        m_jointTransform.setIdentity();
        m_adjPK.setIdentity();
        if (input->dynamics) {
            m_staticFriction = input->dynamics->friction;
            m_viscousFriction = input->dynamics->damping;
        }
        m_optimOriginTransform = m_originTransform;
        Link<T> *t_link = &m_parentLink;
        while (!t_link->isRoot() &&
               ARDL_visit(t_link->getParentJoint(), isFixed())) {
            m_optimOriginTransform.apply(
                ARDL_visit(t_link->getParentJoint(), getOriginTransform()));
            t_link = &(t_link->getParentLink());
        }
    }

    void initDofSizes() {
        m_q.resize(derived().getDof());
        m_q.setZero();
        m_qd.resize(derived().getDof());
        m_qd.setZero();
        m_qdLimit.resize(derived().getDof());
        m_qdLimit.setZero();
        m_qMin.resize(derived().getDof());
        m_qMin.setZero();
        m_qMax.resize(derived().getDof());
        m_qMax.setZero();
    }

    inline T getStaticFriction() { return m_staticFriction; }
    inline T getViscousFriction() { return m_viscousFriction; }

    inline void setFixedTransform(Pose<T> & trans) {
        m_originTransform = trans;
    }

    inline Pose<T> const &getOriginTransform() const {
        return m_originTransform;
    }

    inline Pose<T> &getAdjointLocal() { return m_adPK; }

    inline Pose<T> &getOriginTransformRef() { return m_originTransform; }
    inline void setOptimalFixedTransform(Pose<T> & trans) {
        m_optimOriginTransform = trans;
    }
    inline Pose<T> &getOptimalTransformRef() { return m_optimOriginTransform; }

    inline const Eigen::Matrix<T, 6, 1> &getS() const { return m_s; }

    inline const VectorX<T> &getQ() { return m_q; }
    inline const VectorX<T> &getQd() { return m_qd; }
    inline const VectorX<T> &getVelocity() { return m_qd; }
    inline const VectorX<T> &getMinLimit() { return m_qMin; }

    inline const VectorX<T> &getMaxLimit() { return m_qMax; }

    inline const VectorX<T> &getVelLimit() { return m_qdLimit; }

    inline void setRandomQState() {
        derived().setQ(
            ((((VectorX<T>::Random(derived().getDof()).array() + T(1)) / T(2))
                  .array() *
              (m_qMax - m_qMin).array())
                 .array() +
             m_qMin.array())
                .matrix());
    }
    inline void setRandomQdState() {
        derived().setQd(
            (VectorX<T>::Random(derived().getDof()).array() * m_qdLimit.array())
                .matrix());
    }

    inline Link<T> &getParentLink() { return *m_parentLink; }

#if ARDL_VARIANT

    constexpr inline bool isFixed() { return Derived::isFixed(); };
    constexpr size_t getDof() { return Derived::getDof(); }
    template <typename qDerived>
    inline void setQ(const Eigen::MatrixBase<qDerived> &q) {
        if (m_q != q) {
            m_q = q;
            cache = false;
        } else {
            cache = true;
        }
    }

    template <typename qdDerived>
    inline void setQd(const Eigen::MatrixBase<qdDerived> &qd) {
        if (m_qd != qd) {
            m_qd = qd;
            qd_cache = false;
        } else {
            qd_cache = true;
        }
    }

    inline void updateVelocity() { derived().updateVelocity(); }
#else

    virtual inline bool isFixed() = 0;

    virtual inline size_t getDof() = 0;

    virtual void setQ(const Eigen::Ref<const VectorX<T>> &q) {
        throw std::runtime_error("NOT IMPLEMENTED");
    }

    virtual void setQd(const Eigen::Ref<const VectorX<T>> &qd) {
        if (m_qd != qd) {
            m_qd = qd;
        }
    }
    virtual inline void updateVelocity() = 0;
#endif

    inline Eigen::Matrix<T, 6, 1> &getVelocityVector() {
        return m_adjPK.getVelocity();
    }

    inline const Eigen::Matrix<T, 6, 6> &getAdjPK() const {
        return m_adjPK.getMatrix();
    }

    Motion<T> &getAdjPKRef() { return m_adjPK; }

    inline void update() {
        if (!cache) {
            m_jointTransform.apply(m_originTransform, m_adPK);
        }
        if (!qd_cache)
            updateVelocity();
    }

    inline void updateOptim() {
        if (!cache) {
            m_jointTransform.apply(m_optimOriginTransform, m_adPK);
        }
        if (!qd_cache)
            updateVelocity();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // ARDL_NTYPE_CLASS_BASE(T)
} // namespace ARDL::Model

#pragma once

#include "ARDL/Model/Joint.hpp"

#include <math.h>
using namespace ARDL::Util;
namespace ARDL::Model::Joints {
ARDL_NTYPE_CLASS_INHERIT(T, PrismaticJoint, Joint) {

  public:
    PrismaticJoint() : ARDL_INHERIT_TYPE(T, PrismaticJoint, Joint)() {
        this->initDofSizes();
    }
    PrismaticJoint(urdf::JointSharedPtr input, Link<T> & parentLink)
        : ARDL_INHERIT_TYPE(T, PrismaticJoint, Joint)(input, parentLink) {
        this->initDofSizes();
        this->m_qMin(0) = input->limits->lower;
        this->m_qMax(0) = input->limits->upper;
        this->m_qdLimit(0) = input->limits->velocity;
        this->m_s.setZero();
        this->m_s.template segment<3>(0) << input->axis.x, input->axis.y,
            input->axis.z;
        this->m_s.template head<3>().normalize();
    }
    ~PrismaticJoint() {}

    PrismaticJoint(const PrismaticJoint<T> &copy, Link<T> &parentLink)
        : ARDL_INHERIT_TYPE(T, PrismaticJoint, Joint)(copy, parentLink) {
        this->initDofSizes();
    }
#if ARDL_VARIANT

    static constexpr inline size_t getDof() { return 1; }
    constexpr inline bool isFixed() { return false; }

    template <typename qDerived>
    void setQ(const Eigen::MatrixBase<qDerived> &q) {
        if (this->m_q[0] != q[0]) {

            this->cache = false;
            this->m_q[0] = q[0];
            this->m_jointTransform.getPRef() =
                q[0] * this->m_s.template head<3>();

        } else {
            this->cache = true;
        }
    }

    void updateVelocity() {
        if (!this->qd_cache) {
            this->m_adjPK.setVelocity(this->m_s * this->m_qd(0));
            this->qd_cache = true;
        }
    }
#else

    inline size_t getDof() override { return 1; }
    inline bool isFixed() override { return false; }
    void setQ(const Eigen::Ref<const VectorX<T>> &q) override {
        if (this->m_q[0] != q[0]) {
            this->cache = false;
            this->m_q[0] = q[0];
            this->m_jointTransform.getPRef() =
                q[0] * this->m_s.template head<3>();
        } else {
            this->cache = true;
        }
    }
    void updateVelocity() override {
        if (!this->qd_cache) {
            this->m_adjPK.setVelocity(this->m_s * this->m_qd(0));
            this->qd_cache = true;
        }
    }
#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // ARDL_NTYPE_CLASS_INHERIT(T)
} // namespace ARDL

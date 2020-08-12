#pragma once

#include "ARDL/Model/Joint.hpp"

namespace ARDL {
    namespace Model {
        namespace Joints {
            ARDL_NTYPE_CLASS_INHERIT(T, FixedJoint, Joint) {
                 public:
#if !ARDL_EXTERNAL_DATA
                FixedJoint(urdf::JointSharedPtr input, Link<T>& parentLink)
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)(input, parentLink)
#else
                FixedJoint(urdf::JointSharedPtr input, Link<T>& parentLink, ARDL::Data<T> &data,
                           size_t i)
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)(input, parentLink, data, i)
#endif
                {
                    this->m_s.setZero();
                    this->m_jointTransform.setIdentity();
                    this->initDofSizes();
                    this->update();
                    this->m_adPK= this->m_optimOriginTransform;
                }

#if !ARDL_EXTERNAL_DATA
                FixedJoint()
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)()
#else
                FixedJoint(ARDL::Data<T> &data, size_t i)
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)(data, i)
#endif
                {
                    this->m_s.setZero();
                    this->m_jointTransform.setIdentity();
                    this->initDofSizes();
                    this->update();
                    this->m_adPK= this->m_optimOriginTransform;
                }
#if !ARDL_EXTERNAL_DATA
                FixedJoint(const FixedJoint<T> &copy, Link<T>& parentLink)
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)(copy, parentLink)
#else
                FixedJoint(const FixedJoint<T> &copy, Link<T>& parentLink, ARDL::Data<T> &data,
                           size_t i)
                    : ARDL_INHERIT_TYPE(T, FixedJoint, Joint)(copy, parentLink, data, i)
#endif
                {
                    this->initDofSizes();
                    this->update();
                }
                ~FixedJoint() {}

#if ARDL_VARIANT == ON
                // constexpr inline bool isFixed() { return true; }

                // static constexpr inline size_t getDof() { return 0; }


                static constexpr inline size_t getDof() { return 0; }
                constexpr inline bool isFixed() { return true; }
                template<typename qDerived>
                void setQ(const Eigen::MatrixBase<qDerived> &q) {
                    this->m_jointTransform.setIdentity();
                    this->cache= true;
                }

                template<typename qdDerived>
                void setQd(const Eigen::MatrixBase<qdDerived> &qd) {
                    this->m_adjPK.setIdentity();
                    this->qd_cache= true;
                }
                void updateVelocity() {}
#else
                inline size_t getDof() override { return 0; }
                inline bool isFixed() override { return true; }
                void setQ(const Eigen::Ref<const VectorX<T>> &q) override {
                    this->m_jointTransform.setIdentity();
                    // this->update();
                }
                void setQd(const Eigen::Ref<const VectorX<T>> &qd) override {
                    this->m_adjPK.setIdentity();
                    this->m_qd.setZero();
                }
                void updateVelocity() override {}
#endif
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            }; // ARDL_NTYPE_CLASS_INHERIT(T)
        }      // namespace Joints
    }          // namespace Model
} // namespace ARDL

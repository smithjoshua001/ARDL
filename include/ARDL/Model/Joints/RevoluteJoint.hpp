#pragma once

#include "ARDL/Model/Joint.hpp"

#include <math.h>
using namespace ARDL::Util;
namespace ARDL {
    namespace Model {
        namespace Joints {
            ARDL_NTYPE_CLASS_INHERIT(T, RevoluteJoint, Joint) {
                 private:
                // Eigen::AngleAxis<T> mt_rotationTemp;

                 public:
#if !ARDL_EXTERNAL_DATA
                RevoluteJoint(): ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)() { this->initDofSizes(); }
                RevoluteJoint(urdf::JointSharedPtr input, Link<T>& parentLink)
                    : ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)(input, parentLink)
#else
                RevoluteJoint(ARDL::Data<T> &data, size_t i): ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)(data, i) {
                    this->initDofSizes();
                }
                RevoluteJoint(urdf::JointSharedPtr input, Link<T>& parentLink, ARDL::Data<T> &data,
                              size_t i)
                    : ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)(input, parentLink, data, i)
#endif
                {
                    this->initDofSizes();
                    this->m_qMin(0)= input->limits->lower;
                    this->m_qMax(0)= input->limits->upper;
                    this->m_qdLimit(0)= input->limits->velocity;
                    this->m_s.setZero();
                    this->m_s.template segment<3>(3) << input->axis.x, input->axis.y, input->axis.z;
                    // mt_rotationTemp= Eigen::AngleAxis<T>(1.0, this->m_s.template tail<3>());
                }
                ~RevoluteJoint() {}

#if !ARDL_EXTERNAL_DATA
                RevoluteJoint(const RevoluteJoint<T> &copy, Link<T>& parentLink)
                    : ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)(copy, parentLink)
#else
                RevoluteJoint(const RevoluteJoint<T> &copy, Link<T>& parentLink,ARDL::Data<T> &data, size_t i)
                    : ARDL_INHERIT_TYPE(T, RevoluteJoint, Joint)(copy, parentLink,data, i)
#endif
                {
                    this->initDofSizes();
                    // mt_rotationTemp= copy.mt_rotationTemp;
                }
#if ARDL_VARIANT == ON

                static constexpr inline size_t getDof() { return 1; }
                constexpr inline bool isFixed() { return false; }

                template<typename qDerived>
                void setQ(const Eigen::MatrixBase<qDerived> &q) {
                    if(this->m_q[0] != q[0]) {

                        this->cache= false;
                        this->m_q[0]= q[0];
                        T cosq, sinq;
                        if constexpr(std::is_same<double, T>::value) {
                            sincos(q[0], &sinq, &cosq);
                        } else if constexpr(std::is_same<float, T>::value) {
                            sincosf(q[0], &sinq, &cosq);
                        }
                        T mcosq= 1 - cosq;

                        T *jointTrans= this->m_jointTransform.getRRef().data();
                        jointTrans[0]= cosq + this->m_s(3) * this->m_s(3) * mcosq;
                        jointTrans[1]= this->m_s(4) * this->m_s(3) * mcosq + this->m_s(5) * sinq;
                        jointTrans[2]= this->m_s(5) * this->m_s(3) * mcosq - this->m_s(4) * sinq;
                        jointTrans[3]= this->m_s(3) * this->m_s(4) * mcosq - this->m_s(5) * sinq;
                        jointTrans[4]= cosq + this->m_s(4) * this->m_s(4) * mcosq;
                        jointTrans[5]= this->m_s(5) * this->m_s(4) * mcosq + this->m_s(3) * sinq;
                        jointTrans[6]= this->m_s(3) * this->m_s(5) * mcosq + this->m_s(4) * sinq;
                        jointTrans[7]= this->m_s(4) * this->m_s(5) * mcosq - this->m_s(3) * sinq;
                        jointTrans[8]= cosq + this->m_s(5) * this->m_s(5) * mcosq;
                    } else {
                        this->cache= true;
                    }
                }

                void updateVelocity() {
                    if(!this->qd_cache) {
                        this->m_adjPK.setVelocity(this->m_s * this->m_qd(0));
                        this->qd_cache= true;
                    }
                }
#else

                inline size_t getDof() override { return 1; }
                inline bool isFixed() override { return false; }
                void setQ(const Eigen::Ref<const VectorX<T>> &q) override {
                    if(this->m_q[0] != q[0]) {
                        this->cache= false;
                        this->m_q[0]= q[0];
                        T cosq, sinq;
                        if constexpr(std::is_same<double, T>::value) {
                            sincos(q[0], &sinq, &cosq);
                        } else if constexpr(std::is_same<float, T>::value) {
                            sincosf(q[0], &sinq, &cosq);
                        }
                        T mcosq= 1 - cosq;

                        T *jointTrans= this->m_jointTransform.getRRef().data();
                        jointTrans[0]= cosq + this->m_s(3) * this->m_s(3) * mcosq;
                        jointTrans[1]= this->m_s(4) * this->m_s(3) * mcosq + this->m_s(5) * sinq;
                        jointTrans[2]= this->m_s(5) * this->m_s(3) * mcosq - this->m_s(4) * sinq;
                        jointTrans[3]= this->m_s(3) * this->m_s(4) * mcosq - this->m_s(5) * sinq;
                        jointTrans[4]= cosq + this->m_s(4) * this->m_s(4) * mcosq;
                        jointTrans[5]= this->m_s(5) * this->m_s(4) * mcosq + this->m_s(3) * sinq;
                        jointTrans[6]= this->m_s(3) * this->m_s(5) * mcosq + this->m_s(4) * sinq;
                        jointTrans[7]= this->m_s(4) * this->m_s(5) * mcosq - this->m_s(3) * sinq;
                        jointTrans[8]= cosq + this->m_s(5) * this->m_s(5) * mcosq;
                    } else {
                        this->cache= true;
                    }
                }
                void updateVelocity() override {
                    if(!this->qd_cache) {
                        this->m_adjPK.setVelocity(this->m_s * this->m_qd(0));
                        this->qd_cache= true;
                    }
                }
#endif

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            }; // ARDL_NTYPE_CLASS_INHERIT(T)
        }      // namespace Joints
    }          // namespace Model
} // namespace ARDL

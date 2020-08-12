#pragma once
#include <string>
#include <Eigen/Dense>
#include <urdf_model/model.h>
#include <stdexcept>
#include <variant>

#include "ARDL/Util/Logger.hpp"
#include "ARDL/Math/AdjointSE3.hpp"
#include "ARDL/Math/LieBracketSE3.hpp"
#include "ARDL/Util/Random.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
    template<typename T>
    struct Data;
}

namespace ARDL {
    using namespace Math;
    namespace Model {
        // Forward declare Link such that it can be used in a cyclic fashion.
        template<typename T>
        class Link;

        /**
         * @brief Base type for Joints
         *
         * @tparam T
         */
        ARDL_NTYPE_CLASS_BASE(T, Joint) {
             public:
            ARDL_DERIVED(Joint<T>)

             protected:
#if !ARDL_EXTERNAL_DATA
            Eigen::Matrix<T, 6, 1> m_s;
            // Optimized Transform to origin
            AdjointSE3<T> m_optimOriginTransform;

            // transform to origin
            AdjointSE3<T> m_originTransform;
            // transform from origin to end of joint
            AdjointSE3<T> m_jointTransform;

            // Velocity of Joint
            LieBracketSE3<T> m_adjPK;

            // Transformation of Joint
            AdjointSE3<T> m_adPK, m_adPKOptim;

            T m_staticFriction;
            T m_viscousFriction;

            VectorX<T> m_qMin, m_qMax, m_qdLimit, m_q, m_qd;

            Link<T> &m_parentLink;

            bool cache= false, qd_cache= false;
#else
            Eigen::Matrix<T, 6, 1> &m_s;
            // Optimized Transform to origin
            AdjointSE3<T> &m_optimOriginTransform;

            // transform to origin
            AdjointSE3<T> &m_originTransform;
            // transform from origin to end of joint
            AdjointSE3<T> &m_jointTransform;

            // Velocity of Joint
            LieBracketSE3<T> &m_adjPK;

            // Transformation of Joint
            AdjointSE3<T> &m_adPK, &m_adPKOptim;

            T &m_staticFriction;
            T &m_viscousFriction;

            VectorX<T> &m_qMin, &m_qMax, &m_qdLimit, &m_q, &m_qd;

            Link<T> &m_parentLink;

            char &cache, &qd_cache;
#endif

             public:
#if !ARDL_EXTERNAL_DATA
            Joint(const ARDL_BASE_TYPE(T, Joint) & copy, Link<T> &parentLink)
                : m_parentLink(parentLink)
#else
            Joint(const ARDL_BASE_TYPE(T, Joint) & copy, Link<T> &parentLink, ARDL::Data<T> &data, size_t i)
                : m_s(data.s[i]), m_optimOriginTransform(data.optimOriginTransform[i]),
                  m_originTransform(data.originTransform[i]), m_jointTransform(data.jointTransform[i]),
                  m_adjPK(data.adjPK[i]), m_adPK(data.adPK[i]), m_adPKOptim(data.adPKOptim[i]),
                  m_staticFriction(data.staticFriction[i]), m_viscousFriction(data.viscousFriction[i]),
                  m_qMin(data.qMin[i]), m_qMax(data.qMax[i]), m_qdLimit(data.qdLimit[i]), m_q(data.j_q[i]),
                  m_qd(data.j_qd[i]), m_parentLink(parentLink), cache(data.cache[i]), qd_cache(data.qd_cache[i])
#endif
            {
                m_s= copy.m_s;
                m_optimOriginTransform= copy.m_optimOriginTransform;
                m_originTransform= copy.m_originTransform;
                m_jointTransform= copy.m_jointTransform;
                m_adjPK= copy.m_adjPK;
                m_adPK= copy.m_adPK;
                m_adPKOptim= copy.m_adPKOptim;
                m_staticFriction= copy.m_staticFriction;
                m_viscousFriction= copy.m_viscousFriction;
                m_qMin= copy.m_qMin;
                m_qMax= copy.m_qMax;
                m_qdLimit= copy.m_qdLimit;
                m_q= copy.m_q;
                m_qd= copy.m_qd;
            }
            // Joint(const ARDL_BASE_TYPE(T, Joint) & copy)= delete;

            /****************** INITIALIZATION HANDLING **************/

#if !ARDL_EXTERNAL_DATA
            Joint(urdf::JointSharedPtr input, Link<T> &parentLink)
                : m_parentLink(parentLink)
#else
            Joint(urdf::JointSharedPtr input, Link<T> &parentLink, ARDL::Data<T> &data, size_t i)
                : m_s(data.s[i]), m_optimOriginTransform(data.optimOriginTransform[i]),
                  m_originTransform(data.originTransform[i]), m_jointTransform(data.jointTransform[i]),
                  m_adjPK(data.adjPK[i]), m_adPK(data.adPK[i]), m_adPKOptim(data.adPKOptim[i]),
                  m_staticFriction(data.staticFriction[i]), m_viscousFriction(data.viscousFriction[i]),
                  m_qMin(data.qMin[i]), m_qMax(data.qMax[i]), m_qdLimit(data.qdLimit[i]), m_q(data.j_q[i]),
                  m_qd(data.j_qd[i]), cache(data.cache[i]), qd_cache(data.qd_cache[i]), m_parentLink(parentLink)
#endif
            {
                m_originTransform.getPRef() << input->parent_to_joint_origin_transform.position.x,
                    input->parent_to_joint_origin_transform.position.y,
                    input->parent_to_joint_origin_transform.position.z;
                Eigen::Quaternion<double> t_quat;
                input->parent_to_joint_origin_transform.rotation.getQuaternion(t_quat.x(), t_quat.y(), t_quat.z(),
                                                                               t_quat.w());

                Eigen::Matrix<T, 3, 3> tempRotation= t_quat.toRotationMatrix().cast<T>();

                m_originTransform.setR(tempRotation);

                m_jointTransform.setIdentity();
                m_adjPK.setIdentity();
                if(input->dynamics) {
                    m_staticFriction= input->dynamics->friction;
                    m_viscousFriction= input->dynamics->damping;
                }
                m_optimOriginTransform= m_originTransform;
                Link<T> *t_link= &m_parentLink;
                while(!t_link->isRoot() && ARDL_visit(t_link->getParentJoint(), isFixed())) {
                    m_optimOriginTransform.apply(ARDL_visit(t_link->getParentJoint(), getOriginTransform()));
                    t_link= &(t_link->getParentLink());
                }
                // if(!t_link->isRoot())
                //     ARDL_visit(t_link->getParentJoint(), setOptimalFixedTransform(m_optimOriginTransform));
                // std::cout<<m_optimOriginTransform<<std::endl;
                // if(m_parentLink.getName() =="lwr_arm_9_link" ){
                //     exit(-1);
                // }
            }

            // #if !ARDL_EXTERNAL_DATA
            //             Joint() {
            //                 m_originTransform.setIdentity();
            //                 m_jointTransform.setIdentity();
            //                 m_adjPK.setIdentity();
            //                 m_staticFriction= 0;
            //                 m_viscousFriction= 0;
            //             }
            // #endif
            void initDofSizes() {
                m_q.resize(derived().getDof());
                m_q.setZero();
                m_qd.resize(derived().getDof());
                m_qd.setZero();

                m_q.setConstant(200);
                m_qd.setConstant(200);
                m_qdLimit.resize(derived().getDof());
                m_qdLimit.setZero();
                m_qMin.resize(derived().getDof());
                m_qMin.setZero();
                m_qMax.resize(derived().getDof());
                m_qMax.setZero();
            }

            inline T getStaticFriction() { return m_staticFriction; }
            inline T getViscousFriction() { return m_viscousFriction; }

            inline void setFixedTransform(AdjointSE3<T> &trans) { m_originTransform= trans; }

            inline AdjointSE3<T> const &getOriginTransform() const { return m_originTransform; }

            inline AdjointSE3<T> &getAdjointLocal() { return m_adPK; }

            inline AdjointSE3<T> &getOriginTransformRef() { return m_originTransform; }
            inline void setOptimalFixedTransform(AdjointSE3<T> &trans) { m_optimOriginTransform= trans; }
            inline AdjointSE3<T> & getOptimalTransformRef() { return m_optimOriginTransform; }

            inline const Eigen::Matrix<T, 6, 1> &getS() const { return m_s; }

            inline const VectorX<T> &getQ() { return m_q; }
            inline const VectorX<T> &getQd() { return m_qd; }
            inline const VectorX<T> &getVelocity() { return m_qd; }
            inline const VectorX<T> &getMinLimit() { return m_qMin; }

            inline const VectorX<T> &getMaxLimit() { return m_qMax; }

            inline const VectorX<T> &getVelLimit() { return m_qdLimit; }

            inline void setRandomQState() {
                derived().setQ(((((VectorX<T>::Random(derived().getDof()).array() + T(1)) / T(2)).array() *
                                 (m_qMax - m_qMin).array())
                                    .array() +
                                m_qMin.array())
                                   .matrix());
            }
            inline void setRandomQdState() {
                derived().setQd((VectorX<T>::Random(derived().getDof()).array() * m_qdLimit.array()).matrix());
            }

            inline Link<T> &getParentLink() { return *m_parentLink; }

#if ARDL_VARIANT == ON

            constexpr inline bool isFixed() { return Derived::isFixed(); };
            constexpr size_t getDof() { return Derived::getDof(); }
            template<typename qDerived>
            inline void setQ(const Eigen::MatrixBase<qDerived> &q) {
                std::cout << m_q << std::endl;
                std::cout << q << std::endl;
                if(m_q != q) {
                    m_q= q;
                    cache= false;
                } else {
                    cache= true;
                }
            }

            template<typename qdDerived>
            inline void setQd(const Eigen::MatrixBase<qdDerived> &qd) {
                if(m_qd != qd) {
                    m_qd= qd;
                    qd_cache= false;
                } else {
                    qd_cache= true;
                }
            }

            inline void updateVelocity() { derived().updateVelocity(); }
#else

            virtual inline bool isFixed()= 0;

            virtual inline size_t getDof()= 0;

            virtual void setQ(const Eigen::Ref<const VectorX<T>> &q) { throw std::runtime_error("NOT IMPLEMENTED"); }

            virtual void setQd(const Eigen::Ref<const VectorX<T>> &qd) {
                if(m_qd != qd) { m_qd= qd; }
            }
            virtual inline void updateVelocity()= 0;
#endif

            inline Eigen::Matrix<T, 6, 1> &getVelocityVector() { return m_adjPK.getVelocity(); }

            inline const Eigen::Matrix<T, 6, 6> &getAdjPK() const { return m_adjPK.getMatrix(); }

            LieBracketSE3<T> &getAdjPKRef() { return m_adjPK; }

            inline void update() {
                if(!cache) { m_jointTransform.apply(m_originTransform, m_adPK); }
                if(!qd_cache) updateVelocity();
            }

            inline void updateOptim() {
                if(!cache) { m_jointTransform.apply(m_optimOriginTransform, m_adPK); }
                if(!qd_cache) updateVelocity();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // ARDL_NTYPE_CLASS_BASE(T)
    }      // namespace Model
} // namespace ARDL

#pragma once
#include <string>
#include <Eigen/Dense>
#include <urdf_model/model.h>
#include <stdexcept>

#include "ARDL/Util/Logger.hpp"
#include "ARDL/Math/AdjointSE3.hpp"
#include "ARDL/Math/LieBracketSE3.hpp"
#include "ARDL/Util/Random.hpp"

namespace ARDL {
    using namespace Math;
    namespace Model {
        //Forward declare Link such that it can be used in a cyclic fashion.
        template <typename T> class Link;

        /**
         * @brief Base type for Joints
         *
         * @tparam T
         */
        template <typename T> class Joint {
        protected:

            Eigen::Matrix<T, 6, 1> m_s;
            bool m_isFixed;

            //Optimized Transform to origin
            AdjointSE3<T> m_optimOriginTransform;

            //transform to origin
            AdjointSE3<T> m_originTransform;
            //transform from origin to end of joint
            AdjointSE3<T> m_jointTransform;

            //Velocity of Joint
            LieBracketSE3<T> m_adjPK;

            //Transformation of Joint
            AdjointSE3<T> m_adPK, m_adPKOptim;

            T m_staticFriction;
            T m_viscousFriction;

            T m_qMin, m_qMax, m_qdLimit, m_q, m_qd;

            std::shared_ptr<Link<T> > m_parentLink;

        public:

            Joint(const Joint &copy, std::shared_ptr<Link<T> > parentLink) {
                m_s = copy.m_s;
                m_isFixed = copy.m_isFixed;
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
                m_parentLink = parentLink;
            }
            Joint(const Joint &copy) = delete;
            /****************** INITIALIZATION HANDLING **************/
            Joint(urdf::JointSharedPtr input, std::shared_ptr<Link<T> > parentLink) {
                m_originTransform.getPRef() << input->parent_to_joint_origin_transform.position.x,
                                               input->parent_to_joint_origin_transform.position.y,
                                               input->parent_to_joint_origin_transform.position.z;
                Eigen::Quaternion<double> t_quat;
                input->parent_to_joint_origin_transform.rotation.getQuaternion(t_quat.x(), t_quat.y(), t_quat.z(), t_quat.w());

                Eigen::Matrix<T, 3, 3> tempRotation = t_quat.toRotationMatrix().cast<T>();

                m_originTransform.setR(tempRotation);

                m_jointTransform.setIdentity();
                m_adjPK.setIdentity();
                if (input->dynamics) {
                    m_staticFriction = input->dynamics->friction;
                    m_viscousFriction = input->dynamics->damping;
                }
                m_parentLink = parentLink;
                m_q = 0;
                m_qd = 0;

                m_optimOriginTransform = m_originTransform;
                std::shared_ptr<Link<T> > t_link = m_parentLink;
                while (!t_link->isRoot() && t_link->getParentJoint()->isFixed()) {
                    m_optimOriginTransform.apply(t_link->getParentJoint()->getOriginTransform());
                    t_link = t_link->getParentJoint()->getParentLink();
                }
                m_isFixed = true;
            }

            Joint() {
                m_originTransform.setIdentity();
                m_jointTransform.setIdentity();
                m_adjPK.setIdentity();
                m_staticFriction = 0;
                m_viscousFriction = 0;
            }

            virtual ~Joint() {}

            bool isFixed() {
                return m_isFixed;
            }

            T getStaticFriction() {
                return m_staticFriction;
            }
            T getViscousFriction() {
                return m_viscousFriction;
            }

            void setFixedTransform(AdjointSE3<T> &trans) {
                m_originTransform = trans;
            }

            AdjointSE3<T> const &getOriginTransform() const {
                return m_originTransform;
            }

            AdjointSE3<T> &getOriginTransformRef() {
                return m_originTransform;
            }

            Eigen::Matrix<T, 6, 1> const &getS() const {
                return m_s;
            }

            T getMinLimit() {
                return m_qMin;
            }

            T getMaxLimit() {
                return m_qMax;
            }

            T getVelLimit() {
                return m_qdLimit;
            }

            void setRandomQState() {
                m_q = (T(ARDL::Util::Random::rand_dist(ARDL::Util::Random::e2)) * (m_qMax - m_qMin)) + m_qMin;
                setQ(m_q);
            }
            void setRandomQdState() {
                m_qd = (T(ARDL::Util::Random::rand_dist(ARDL::Util::Random::e2)) * (m_qdLimit * 2)) - m_qdLimit;
                setQd(m_qd);
            }
            const T &getVelocity() {
                return m_qd;
            }
            const T &getQd() {
                return m_qd;
            }

            const T &getQ() {
                return m_q;
            }

            std::shared_ptr<Link<T> > getParentLink() {
                return m_parentLink;//.lock();
            }

            virtual void setQ(const T &q) {
                throw std::runtime_error("NOT IMPLEMENTED");
            }

            virtual void setQd(T qd) {
                // m_adjPK.setVelocity(m_s * qd);
                m_qd = qd;
            }

            Eigen::Matrix<T, 6, 1> &getVelocityVector() {
                return m_adjPK.getVelocity();
            }

            const Eigen::Matrix<T, 6, 6> &getAdjPK() const {
                return m_adjPK.getMatrix();
            }

            LieBracketSE3<T> &getAdjPKRef() {
                return m_adjPK;
            }

            void update() {
                m_adPK = m_originTransform;
                m_jointTransform.apply(m_adPK, m_adPK);
                m_adjPK.setVelocity(m_s * m_qd);
            }

            void updateOptim() {
                m_adPK = m_optimOriginTransform;
                m_jointTransform.apply(m_adPK, m_adPK);
                m_adjPK.setVelocity(m_s * m_qd);
            }

            AdjointSE3<T> &getAdjointLocal() {
                return m_adPK;
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

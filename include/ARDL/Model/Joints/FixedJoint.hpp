#pragma once

#include "ARDL/Model/Joint.hpp"

namespace ARDL {
    namespace Model {
        namespace Joints {
            template<typename T> class FixedJoint : public Joint<T> {
            private:
            public:
                FixedJoint(urdf::JointSharedPtr input, std::shared_ptr<Link<T> > parentLink) : Joint<T>(input, parentLink) {
                    this->m_s.setZero();
                    this->m_isFixed = true;
                    this->m_jointTransform.setIdentity();
                    this->update();
                }
                FixedJoint() : Joint<T>() {
                    this->m_s.setZero();
                    this->m_isFixed = true;
                    this->m_jointTransform.setIdentity();
                    this->update();
                }
                FixedJoint(const FixedJoint<T> &copy, std::shared_ptr<Link<T> > parentLink) : Joint<T>(copy, parentLink) {}
                virtual ~FixedJoint() {}

                void setQ(const T &q) override {
                    this->m_jointTransform.setIdentity();
                    // this->update();
                }
                void setQd(T qd) override {
                    this->m_adjPK.setIdentity();
                    this->m_qd = 0;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };
        }
    }
}

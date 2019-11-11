#pragma once

#include "ARDL/Model/Joint.hpp"

namespace ARDL {
    namespace Model {
        namespace Joints {
            template<typename T> class RevoluteJoint : public Joint<T> {
            private:
                Eigen::AngleAxis<T> mt_rotationTemp;
            public:
                RevoluteJoint(urdf::JointSharedPtr input, std::shared_ptr<Link<T> > parentLink) : Joint<T>(input, parentLink) {
                    this->m_isFixed = false;
                    this->m_qMin = input->limits->lower;
                    this->m_qMax = input->limits->upper;
                    this->m_qdLimit = input->limits->velocity;
                    this->m_s.setZero();
                    this->m_s.template segment<3>(3) << input->axis.x, input->axis.y, input->axis.z;
                    this->m_s.template segment<3>(3) = this->m_s.template segment<3>(3);
                    mt_rotationTemp = Eigen::AngleAxis<T>(1.0, this->m_s.template tail<3>());
                }
                virtual ~RevoluteJoint() {}
                RevoluteJoint(const RevoluteJoint<T> &copy, std::shared_ptr<Link<T> > parentLink) : Joint<T>(copy, parentLink) {
                    mt_rotationTemp = copy.mt_rotationTemp;
                }

                void setQ(const T &q) override {
                    this->m_q = q;
                    mt_rotationTemp.angle() = q;
                    this->m_jointTransform.setR(mt_rotationTemp.toRotationMatrix());
                    // this->update();
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };
        }
    }
}

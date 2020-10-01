#pragma once
#include "ARDL/Constraints/Constraint.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
    using namespace Model;
    namespace Constraints {
        template <typename T> class JointVelocityLimit : public Constraint<T, JointVelocityLimit<T> > {
        protected:
            ARDL::VectorX<T> m_qdLimit;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            template <typename Derived> JointVelocityLimit(std::shared_ptr<Chain<T> > model, const Eigen::MatrixBase<Derived> &qdLimit) : Constraint<T, JointVelocityLimit<T> >(model->getNumOfJoints(), model) {
                m_qdLimit.resize(model->getNumOfJoints());
                m_qdLimit = qdLimit;
            }

            bool compute() {
                this->m_constraintVec = this->m_model_sp->getQd().array().abs().matrix() - m_qdLimit;
                return (this->m_constraintVec.array() >= 0).any();
            }

            template <typename Derived> void setLimit(const Eigen::MatrixBase<Derived> &qdLimit) {
                m_qdLimit = qdLimit;
            }
        };
    }
}

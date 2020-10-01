#pragma once
#include "ARDL/Constraints/Constraint.hpp"
#include "ARDL/Kinematics/ForwardKinematics.hpp"

namespace ARDL {
    using namespace Model;
    namespace Constraints {
        template <typename T> class SelfCollision : public Constraint<T, SelfCollision<T> > {
        protected:
            std::shared_ptr<ForwardKinematics<T> > m_kin_sp;
        public:
            SelfCollision(std::shared_ptr<Chain<T> > model, std::shared_ptr<ForwardKinematics<T> > kin) : Constraint<T, SelfCollision<T> >(1, model), m_kin_sp(kin) {}
            bool compute() {
                this->m_constraintVec(0) = (T)m_kin_sp->collisionOptim();
                return (bool)this->m_constraintVec(0);
            }
        };
    }
}

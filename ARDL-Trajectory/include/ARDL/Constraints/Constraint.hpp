#pragma once
#include <Eigen/Dense>
#include "ARDL/Model/Chain.hpp"

namespace ARDL {
    using namespace Model;
    namespace Constraints {
        template <typename T, class Derived> class Constraint {
        private:
            Derived &derived() {return *static_cast<Derived *>(this);}
        protected:
            std::shared_ptr<Chain<T> > m_model_sp;
            size_t m_numConstraintParams;
            bool m_enabled = true;
            VectorX<T> m_constraintVec;
        public:
            Constraint(size_t numConstraints, std::shared_ptr<Chain<T> > model) : m_model_sp(model), m_numConstraintParams(numConstraints) {
                m_constraintVec.resize(getNumOfConstraintParams());
            }

            size_t getNumOfConstraintParams() {
                return m_numConstraintParams;
            }
            void setEnabled(bool en) {
                m_enabled = en;
            }
            bool compute() {
                return (!m_enabled && m_constraintVec.setZero()) || derived().compute();
                // if (m_enabled)
                //     return derived().compute();
                // else
                //     return true;
            }

            const VectorX<T> &getConstraintVector() {
                return m_constraintVec;
            }
        };
    }
}

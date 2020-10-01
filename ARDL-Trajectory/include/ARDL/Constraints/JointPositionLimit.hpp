#pragma once
#include "ARDL/Constraints/Constraint.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
    using namespace Model;
    namespace Constraints {
        template <typename T> class JointPositionLimit : public Constraint<T, JointPositionLimit<T> > {
        protected:
            ARDL::VectorX<T> m_qMax, m_qMin;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            JointPositionLimit(std::shared_ptr<Chain<T> > model) : Constraint<T, JointPositionLimit<T> >(model->getNumOfJoints() * 2, model) {
                std::vector<std::pair<T, T> > t_limits;
                t_limits.resize(model->getNumOfJoints());
                m_qMax.resize(model->getNumOfJoints());
                m_qMin.resize(model->getNumOfJoints());

                model->getJointLimits(t_limits);
                for (size_t i = 0; i < model->getNumOfJoints(); i++) {
                    m_qMax(i) = t_limits[i].second;
                    m_qMin(i) = t_limits[i].first;
                }
            }

            template <typename Derived> JointPositionLimit(std::shared_ptr<Chain<T> > model, const Eigen::MatrixBase<Derived> &qMin, const Eigen::MatrixBase<Derived> &qMax) : Constraint<T, JointPositionLimit<T> >(model->getNumOfJoints() * 2, model) {
                m_qMax.resize(model->getNumOfJoints());
                m_qMin.resize(model->getNumOfJoints());
                m_qMax = qMax;
                m_qMin = qMin;
            }

            bool compute() {
                this->m_constraintVec.head(this->m_model_sp->getNumOfJoints()) = m_qMin - this->m_model_sp->getQ();
                this->m_constraintVec.tail(this->m_model_sp->getNumOfJoints()) = this->m_model_sp->getQ() - m_qMax;
                return !((this->m_model_sp->getQ().array() >= m_qMin.array()).all() && (this->m_model_sp->getQ().array() <= m_qMax.array()).all());
            }

            template <typename Derived> void setMin(const Eigen::MatrixBase<Derived> &qMin) {
                m_qMin = qMin;
            }

            template <typename Derived> void setMax(const Eigen::MatrixBase<Derived> &qMax) {
                m_qMax = qMax;
            }
        };
    }
}

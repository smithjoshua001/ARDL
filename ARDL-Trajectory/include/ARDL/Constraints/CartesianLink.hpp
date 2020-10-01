#pragma once
#include "ARDL/Constraints/Constraint.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
    using namespace Model;
    namespace Constraints {
        template <typename T> class CartesianLink : public Constraint<T, CartesianLink<T> > {
        protected:
            const aligned_vector<AdjointSE3<T> > &m_adjoints_r;
            Eigen::Matrix<T, 3, Eigen::Dynamic> m_minLinks, m_maxLinks;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            template <typename Derived> CartesianLink(std::shared_ptr<Chain<T> > model, const aligned_vector<AdjointSE3<T> > &adjoints, const Eigen::MatrixBase<Derived> &min, const Eigen::MatrixBase<Derived> &max) : Constraint<T, CartesianLink<T> >(adjoints.size() * 3 * 2, model), m_adjoints_r(adjoints) {
                // m_adjoints_r = adjoints;
                m_minLinks = min;
                m_maxLinks = max;
            }

            bool compute() {
                for (size_t i = 0; i < m_adjoints_r.size(); i++) {
                    this->m_constraintVec.template segment<3>(6 * i) = (m_adjoints_r[i].getP() - m_maxLinks.col(i));
                    this->m_constraintVec.template segment<3>(6 * i + 3) = m_minLinks.col(i) - m_adjoints_r[i].getP();
                }
                return (this->m_constraintVec.array() >= 0).any();
            }
        };
    }
}

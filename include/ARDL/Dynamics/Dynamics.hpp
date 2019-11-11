#pragma once

#include "ARDL/typedefs.hpp"
#include "ARDL/Model/Chain.hpp"
#include "ARDL/Kinematics/ForwardKinematics.hpp"
#include "ARDL/Util/Math.hpp"
#include "ARDL/Util/Filter.hpp"
#include "ARDL/Dynamics/LinkRegressors.hpp"
namespace ARDL {
    using namespace Model;
    using namespace Math;
    using namespace Util::Filters;
    using namespace Util::Math;
    using namespace Regressors;
    template<typename T> class Dynamics {
    private:
        std::shared_ptr<Chain<T> > m_chain;

        Gravity<T> m_gravity, mt_gravity;

        LieBracketSE3<T> mt_bodyVelocity, mt_tempVelocity;

        LinkRegressor<T> mt_momentumRegressor, mt_momentumRegressor2;

        size_t m_baseParamNo;

        Regressor<T> m_projRegressor, m_projParameters;

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mt_6xJ;

        Eigen::Matrix<T, 6, 6>  mt_6x6_1, mt_6x6_2;

    public:
        Dynamics(const Dynamics<T> &copy, std::shared_ptr<Chain<T> > chain) {
            m_chain = chain;
            m_gravity = copy.m_gravity;
            mt_gravity = copy.mt_gravity;
            mt_bodyVelocity = copy.mt_bodyVelocity;
            mt_tempVelocity = copy.mt_tempVelocity;
            mt_momentumRegressor = copy.mt_momentumRegressor;
            mt_momentumRegressor2 = copy.mt_momentumRegressor2;
            m_baseParamNo = copy.m_baseParamNo;
            m_projRegressor = copy.m_projRegressor;
            m_projParameters = copy.m_projParameters;
            mt_6xJ = copy.mt_6xJ;
            mt_6x6_1 = copy.mt_6x6_1;
            mt_6x6_2 = copy.mt_6x6_2;
        }
        Dynamics(std::shared_ptr<Chain<T> > &chain, Gravity<T> &grav) {
            this->m_chain = chain;
            this->m_gravity = grav;
            mt_6xJ.resize(6, this->m_chain->getNumOfJoints());
        }

        Dynamics(std::shared_ptr<Chain<T> > &chain) {
            this->m_chain = chain;
            this->m_gravity.setZero();
            this->m_gravity(2) = 9.81;
            mt_6xJ.resize(6, this->m_chain->getNumOfJoints());
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        std::shared_ptr<Chain<T> > &getChain() {
            return m_chain;
        }

        /**
         * @brief Set acceleration due to gravity
         *
         * @param gravity
         */
        void setGravity(const Gravity<T> &gravity) {
            this->m_gravity = gravity;
        }

        /**
         * @brief Calculate the Joint Inertia matrix
         *
         * @tparam InertiaD Subtype of inertia matrix (allow for block)
         * @param jacobians Aligned vector of jacobians
         * @param M Inertia matrix
         */
        template<typename InertiaD> void calcJointInertiaMatrix(const aligned_vector<Jacobian<T> > &jacobians, Eigen::MatrixBase<InertiaD> const &M) {
            size_t i = 0;
            const_cast<Eigen::MatrixBase<InertiaD> &>(M).setZero();
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    mt_6xJ.noalias() = link->getSpatialInertia() * jacobians[i];
                    const_cast<Eigen::MatrixBase<InertiaD> &>(M).noalias() += jacobians[i].transpose() * mt_6xJ;
                    i++;
                }
            }
        }

        /**
         * @brief Calculate Coriolis and centrifugal matrix
         *
         * @tparam Derived Subtype of output matrix
         * @param jacobians Aligned Vector of jacobians
         * @param jacobianDots Aligned vector of the derivative of the jacobians
         * @param lbSE3 Lie Bracket for each joint
         * @param C Output Coriolis and centrifugal matrix
         */
        template<typename Derived> void calcCoriolisMatrix(aligned_vector<Jacobian<T> > &jacobians, aligned_vector<Jacobian<T> > &jacobianDots, aligned_vector<LieBracketSE3<T> > &lbSE3, Eigen::MatrixBase<Derived> const &C) {
            const_cast<Eigen::MatrixBase<Derived> &>(C).setZero();
            size_t i = 0;
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    mt_6x6_1.noalias() = link->getSpatialInertia() * lbSE3[i].getMatrix();
                    mt_6x6_2.noalias() = mt_6x6_1 - mt_6x6_1.transpose();
                    mt_6xJ.noalias() = mt_6x6_2 * jacobians[i];
                    mt_6xJ.noalias() += link->getSpatialInertia() * jacobianDots[i];

                    const_cast<Eigen::MatrixBase<Derived> & >(C).noalias() += jacobians[i].transpose() * mt_6xJ;
                    i++;
                }
            }
        }

        /**
         * @brief Calculate Gravity vector
         *
         * @tparam Derived Subtype of gravity vector
         * @param jacobians aligned vector of jacobians
         * @param Ads aligned vector of Adjoint transforms
         * @param G Output gravity vector
         */
        template<typename Derived> void calcGravityVector(aligned_vector<Jacobian<T> > &jacobians, aligned_vector<AdjointSE3<T> > &Ads, Eigen::MatrixBase<Derived> const &G) {
            const_cast<Eigen::MatrixBase<Derived> & >(G).setZero();
            size_t i = 0;
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    Ads[i].applyInverse(m_gravity, mt_gravity);
                    mt_6x6_1.col(0).noalias() = link->getSpatialInertia() * mt_gravity;
                    const_cast<Eigen::MatrixBase<Derived> & >(G).noalias() += jacobians[i].transpose() * mt_6x6_1.col(0);
                    i++;
                }
            }
        }

        template<typename Derived> void calcFrictionVector(Eigen::MatrixBase<Derived> const &F, const T eps = T(1e-30)) {
            const_cast<Eigen::MatrixBase<Derived> & >(F).setZero();
            size_t i = 0;
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    const_cast<Eigen::MatrixBase<Derived> & >(F)(i) = link->getInertialParams()[10] * m_chain->getQd()(i) + link->getInertialParams()[11] * sgn(m_chain->getQd()(i), eps);
                    i++;
                }
            }
        }

        template<typename RegressorD, typename JointD> void calcSlotineLiRegressor(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<LieBracketSE3<T> > &lbSE3, const aligned_vector<AdjointSE3<T> > &Ads, const Eigen::MatrixBase<JointD> &qd, const Eigen::MatrixBase<JointD> &qdd, Eigen::MatrixBase<RegressorD> const &out, const T eps = T(1e-30)) {
            const_cast<Eigen::MatrixBase<RegressorD> & >(out).setZero();
            size_t regressorCols = out.cols() / m_chain->getNumOfJoints();
            for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                Ads[i].applyInverse(m_gravity, mt_bodyVelocity.getVelocity());
                mt_bodyVelocity.getVelocity().noalias() += jacobians[i] * qdd;
                mt_6x6_1.col(0).noalias() = jacobians[i] * qd;
                mt_bodyVelocity.getVelocity().noalias() += lbSE3[i].getMatrix() * mt_6x6_1.col(0);
                mt_bodyVelocity.getVelocity().noalias() += jacobianDots[i] * qd;
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                momentumRegressor(jacobians[i] * qd, mt_momentumRegressor2);
                mt_momentumRegressor.noalias() -= lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2;
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10).noalias() = jacobians[i].transpose() * mt_momentumRegressor;
                if (regressorCols == 12) {
                    //limit creates a deadzone for qd around 0 (might help with qd noise)
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 10) = deadzone(m_chain->getQd()(i), eps) * m_chain->getQd()(i);
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 11) = sgn(m_chain->getQd()(i), eps);
                }
            }
        }

        template<typename RegressorD, typename JointD> void initFilteredSlotineLiRegressor(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<LieBracketSE3<T> > &lbSE3, const aligned_vector<AdjointSE3<T> > &Ads, const Eigen::MatrixBase<JointD> &qd, DiscreteLowPassFilter<T> &filter, Eigen::MatrixBase<RegressorD> const &out, const T eps = T(1e-30)) {
            const_cast<Eigen::MatrixBase<RegressorD> & >(out).setZero();
            size_t regressorCols = out.cols() / m_chain->getNumOfJoints();
            for (int i = 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                Ads[i].applyInverse(m_gravity, mt_bodyVelocity.getVelocity());
                mt_bodyVelocity.getVelocity() = (filter.getBeta() * jacobians[i] * qd - lbSE3[i].getMatrix() * jacobians[i] * qd) - mt_bodyVelocity.getVelocity();
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                momentumRegressor(jacobians[i] * qd, mt_momentumRegressor2);
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * (mt_momentumRegressor + lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2);
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) += jacobianDots.at(i).transpose() * mt_momentumRegressor2;
                if (regressorCols == 12) {
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 10) = -deadzone(m_chain->getQd()(i), eps) * m_chain->getQd()(i);
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 11) = -sgn(m_chain->getQd()(i), eps);
                }
            }
            filter.setBuffer(out);
            const_cast<Eigen::MatrixBase<RegressorD> & >(out).setZero();
            for (int i = 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                mt_bodyVelocity.getVelocity() = filter.getBeta() * jacobians[i] * qd;
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * (mt_momentumRegressor);
            }
            const_cast<Eigen::MatrixBase<RegressorD> &>(out) -= (filter.getResult());
        }

        template<typename RegressorD, typename JointD> void computeFilteredSlotineLiRegressor(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<LieBracketSE3<T> > &lbSE3, const aligned_vector<AdjointSE3<T> > &Ads, const Eigen::MatrixBase<JointD> &qd, DiscreteLowPassFilter<T> &filter, Eigen::MatrixBase<RegressorD> const &out, const T eps = T(1e-30)) {
            const_cast<Eigen::MatrixBase<RegressorD> & >(out).setZero();
            size_t regressorCols = out.cols() / m_chain->getNumOfJoints();
            for (int i = 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                Ads[i].applyInverse(m_gravity, mt_bodyVelocity.getVelocity());
                mt_bodyVelocity.getVelocity() = (filter.getBeta() * jacobians[i] * qd - lbSE3[i].getMatrix() * jacobians[i] * qd) - mt_bodyVelocity.getVelocity();
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                momentumRegressor(jacobians[i] * qd, mt_momentumRegressor2);
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * (mt_momentumRegressor + lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2);
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) += jacobianDots[i].transpose() * mt_momentumRegressor2;
                if (regressorCols == 12) {
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 10) = -deadzone(m_chain->getQd()(i), eps) * m_chain->getQd()(i);
                    const_cast<Eigen::MatrixBase<RegressorD> &>(out)(i, regressorCols *i + 11) = -sgn(m_chain->getQd()(i), eps);
                }
            }
            filter.compute(out);
            const_cast<Eigen::MatrixBase<RegressorD> & >(out).setZero();
            for (int i = 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                mt_bodyVelocity.getVelocity() = filter.getBeta() * jacobians[i] * qd;
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                const_cast<Eigen::MatrixBase<RegressorD> & >(out).block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * (mt_momentumRegressor);
            }
            const_cast<Eigen::MatrixBase<RegressorD> &>(out) -= (filter.getResult());
        }

        void calcBaseProjection(size_t random_samples, T qddLimit = 1, T threshold = T(1e-4), T eps = T(1e-30)) {
            Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic> regressors(random_samples * m_chain->getNumOfJoints(), 12 * m_chain->getNumOfJoints());
            Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic> R(12 * m_chain->getNumOfJoints(), 12 * m_chain->getNumOfJoints());
            ForwardKinematics<T> fk(m_chain);
            aligned_vector<AdjointSE3<T> > Ads(m_chain->getNumOfJoints());
            aligned_vector<LieBracketSE3<T> > lbSE3(m_chain->getNumOfJoints());
            aligned_vector<Eigen::Matrix<T, 6, Eigen::Dynamic> > jacobians, jacobianDots;
            jacobians.resize(m_chain->getNumOfJoints());
            jacobianDots.resize(m_chain->getNumOfJoints());
            for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
                jacobians.at(i).resize(6, m_chain->getNumOfJoints());
                jacobians.at(i).setZero();
                jacobianDots.at(i).resize(6, m_chain->getNumOfJoints());
                jacobianDots.at(i).setZero();
            }
            Eigen::Matrix<T, Eigen::Dynamic, 1> qd, qdd;
            qd.resize(m_chain->getNumOfJoints());
            qdd.resize(m_chain->getNumOfJoints());
            R.setZero();
            // CREATE RANDOM REGRESSOR STACK
            for (size_t k = 0; k < random_samples; k++) {
                m_chain->random();
                fk.getAdjoints(Ads);
                fk.getBodyJacobian(jacobians, jacobianDots);
                qd = m_chain->getQd();
                qdd = Eigen::Matrix<T, Eigen::Dynamic, 1>::Random(m_chain->getNumOfJoints());
                qdd *= qddLimit;
                fk.getLieBrackets(lbSE3, jacobians);
                this->calcSlotineLiRegressor(jacobians, jacobianDots, lbSE3, Ads, qd, qdd, regressors.block(k * m_chain->getNumOfJoints(), 0, m_chain->getNumOfJoints(), 12 * m_chain->getNumOfJoints()), eps);
                R += regressors.block(k * m_chain->getNumOfJoints(), 0, m_chain->getNumOfJoints(), 12 * m_chain->getNumOfJoints()).transpose() * regressors.block(k * m_chain->getNumOfJoints(), 0, m_chain->getNumOfJoints(), 12 * m_chain->getNumOfJoints());
                // std::cout << R << std::endl;
            }
            Eigen::ColPivHouseholderQR < Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > qr(R);
            qr.setThreshold(threshold);
            qr.compute(R);

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Q(m_chain->getNumOfJoints() * 12, m_chain->getNumOfJoints() * 12);
            Q.setZero();
            Q = qr.householderQ();

            Eigen::Matrix<T, -1, -1> RQ(m_chain->getNumOfJoints() * 12, m_chain->getNumOfJoints() * 12);
            RQ.setZero();
            RQ = qr.matrixR();

            Eigen::Matrix<T, -1, -1> PQ(m_chain->getNumOfJoints() * 12, m_chain->getNumOfJoints() * 12);
            PQ.setZero();
            PQ = qr.colsPermutation();

            m_baseParamNo = qr.rank();

            m_projRegressor = PQ.block(0, 0, PQ.rows(), m_baseParamNo);

            m_projParameters = RQ.block(0, 0, m_baseParamNo, m_baseParamNo).inverse() * RQ.block(0, m_baseParamNo, m_baseParamNo, RQ.cols() - m_baseParamNo);

            m_projParameters = (m_projParameters.array().abs() > 1e-4).matrix().template cast<T>().cwiseProduct(m_projParameters);

            m_projParameters = m_projRegressor.transpose() + m_projParameters * PQ.block(0, m_baseParamNo, PQ.rows(), PQ.cols() - m_baseParamNo).transpose();
        }

        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &getRegressorProjector() {
            return m_projRegressor;
        }
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &getParameterProjector() {
            return m_projParameters;
        }
        const size_t &getNumOfBaseParams() {
            return m_baseParamNo;
        }

        template<int j> void calcJointInertiaMatrixDq(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, aligned_vector<Eigen::Matrix<T, j, j> > &M) {
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                size_t i = 0;
                M[i1].setZero();
                for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                    if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                        M[i1] += jacobiansDq[i][i1].transpose() * link->getSpatialInertia() * jacobians[i] + jacobians[i].transpose() * link->getSpatialInertia() * jacobiansDq[i][i1];
                        i++;
                    }
                }
            }
        }

        template<int j> void calcCoriolisMatrixDq(const aligned_vector<LieBracketSE3<T> > &lbSE3, const aligned_vector<Jacobian<T> > &jacobians, aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobianDotsDq, aligned_vector<Eigen::Matrix<T, j, j> > &C) {
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                size_t i = 0;
                for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                    if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                        mt_bodyVelocity.setVelocity(jacobiansDq[i][i1] * m_chain->getQd());

                        // (link->getSpatialInertia() * lbSE3[i].getMatrix() - lbSE3[i].getMatrix().transpose() * link->getSpatialInertia())

                        C[i1] += jacobiansDq[i][i1].transpose() * ((link->getSpatialInertia() * lbSE3[i].getMatrix() - lbSE3[i].getMatrix().transpose() * link->getSpatialInertia()) * jacobians[i] + link->getSpatialInertia() * jacobianDots[i]) + jacobians[i].transpose() * link->getSpatialInertia() * jacobianDotsDq[i][i1] + jacobians[i].transpose() * ((link->getSpatialInertia() * mt_bodyVelocity.getMatrix() - mt_bodyVelocity.getMatrix().transpose() * link->getSpatialInertia()) * jacobians[i] + (link->getSpatialInertia() * lbSE3[i].getMatrix() - lbSE3[i].getMatrix().transpose() * link->getSpatialInertia()) * jacobiansDq[i][i1]);

                        //jacobians[i].transpose() * ((link->getSpatialInertia() * adjuncts[i].getMatrix() - adjuncts[i].getMatrix().transpose() * link->getSpatialInertia()) * jacobians[i] + link->getSpatialInertia() * jacobianDots[i]);
                        i++;
                    }
                }
            }
        }

        template<int j> void calcCoriolisMatrixDqd(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, aligned_vector<Eigen::Matrix<T, j, j> > &C) {
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                size_t i = 0;
                for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                    if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                        mt_bodyVelocity.setVelocity(jacobians[i].col(i1));

                        // (link->getSpatialInertia() * lbSE3[i].getMatrix() - lbSE3[i].getMatrix().transpose() * link->getSpatialInertia())

                        C[i1] += jacobians[i].transpose() * ((link->getSpatialInertia() * mt_bodyVelocity.getMatrix() - mt_bodyVelocity.getMatrix().transpose() * link->getSpatialInertia()) * jacobians[i] + link->getSpatialInertia() * jacobiansDq[i][i1]);

                        //jacobians[i].transpose() * ((link->getSpatialInertia() * adjuncts[i].getMatrix() - adjuncts[i].getMatrix().transpose() * link->getSpatialInertia()) * jacobians[i] + link->getSpatialInertia() * jacobianDots[i]);
                        i++;
                    }
                }
            }
        }

        template<int j> void calcGravityVectorDq(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, const aligned_vector<AdjointSE3<T> > &Ads, aligned_vector<Eigen::Matrix<T, j, 1> > &G) {
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                G[i1].setZero();
                size_t i = 0;
                for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                    if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                        Ads[i].applyInverse(m_gravity, mt_gravity);
                        mt_bodyVelocity.setVelocity(jacobians[i].col(i1));
                        G[i1] += (jacobiansDq[i][i1].transpose() * link->getSpatialInertia() - jacobians[i].transpose() * link->getSpatialInertia() * mt_bodyVelocity.getMatrix()) * mt_gravity;
                        i++;
                    }
                }
            }
        }

        template<typename JointD> void calcSlotineLiRegressorDq(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobianDotsDq, const aligned_vector<LieBracketSE3<T> > &lbSE3, const aligned_vector<AdjointSE3<T> > &Ads, const Eigen::MatrixBase<JointD> &qd, const Eigen::MatrixBase<JointD> &qdd, aligned_vector<Regressor<T> > &regressorsDq, const T eps = T(1e-4)) {
            size_t regressorCols = regressorsDq[0].cols() / m_chain->getNumOfJoints();
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                regressorsDq[i1].setZero();
                for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
                    mt_bodyVelocity.getVelocity().setZero();
                    Ads[i].applyInverse(m_gravity, mt_bodyVelocity.getVelocity());
                    mt_bodyVelocity.getVelocity() += jacobians[i] * qdd + lbSE3[i].getMatrix() * jacobians[i] * qd + jacobianDots[i] * qd;
                    momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                    momentumRegressor(jacobians[i] * qd, mt_momentumRegressor2);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobiansDq[i][i1].transpose() * (mt_momentumRegressor - lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2);

                    Ads[i].applyInverse(m_gravity, mt_bodyVelocity.getVelocity());

                    mt_tempVelocity.setVelocity(jacobians[i].col(i1));
                    mt_bodyVelocity.getVelocity() = -mt_tempVelocity.getMatrix() * mt_bodyVelocity.getVelocity() + jacobiansDq[i][i1] * qdd + jacobianDotsDq[i][i1] * qd + lbSE3[i].getMatrix() * jacobiansDq[i][i1] * qd;

                    mt_tempVelocity.setVelocity(jacobiansDq[i][i1] * m_chain->getQd());
                    mt_bodyVelocity.getVelocity() += mt_tempVelocity.getMatrix() * jacobians[i] * qd;
                    momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) += jacobians[i].transpose() * mt_momentumRegressor;

                    momentumRegressor(jacobiansDq[i][i1] * m_chain->getQd(), mt_momentumRegressor);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) -= jacobians[i].transpose() * (mt_tempVelocity.getMatrix().transpose() * mt_momentumRegressor2 + lbSE3[i].getMatrix().transpose() * mt_momentumRegressor);

                    if (regressorCols == 12) {
                        //limit creates a deadzone for qd around 0 (might help with qd noise)
                        regressorsDq[i1](i, regressorCols *i + 10) = 0;
                        regressorsDq[i1](i, regressorCols *i + 11) = 0;
                    }
                }
            }
        }

        template<typename JointD> void calcSlotineLiRegressorDqd(const aligned_vector<Jacobian<T> > &jacobians, const aligned_vector<Jacobian<T> > &jacobianDots, const aligned_vector<aligned_vector<Jacobian<T> > > &jacobiansDq, const aligned_vector<LieBracketSE3<T> > &lbSE3, const Eigen::MatrixBase<JointD> &qd, aligned_vector<Regressor<T> > &regressorsDq, const T eps = T(1e-4)) {
            size_t regressorCols = regressorsDq[0].cols() / m_chain->getNumOfJoints();
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                regressorsDq[i1].setZero();
                for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
                    mt_bodyVelocity.getVelocity().setZero();
                    mt_tempVelocity.setVelocity(jacobians[i].col(i1));
                    mt_bodyVelocity.getVelocity() = mt_tempVelocity.getMatrix() * jacobians[i] * qd + lbSE3[i].getMatrix() * jacobians[i].col(i1) + jacobiansDq[i][i1] * qd + jacobianDots[i].col(i1);
                    momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                    momentumRegressor(jacobians[i] * qd, mt_momentumRegressor2);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * (mt_momentumRegressor - mt_tempVelocity.getMatrix().transpose() * mt_momentumRegressor2);

                    momentumRegressor(jacobians[i].col(i1), mt_momentumRegressor);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) -= jacobians[i].transpose() * lbSE3[i].getMatrix().transpose() * mt_momentumRegressor;
                }
                if (regressorCols == 12) {
                    //limit creates a deadzone for qd around 0 (might help with qd noise)
                    regressorsDq[i1](i1, regressorCols *i1 + 10) = 1;
                    regressorsDq[i1](i1, regressorCols *i1 + 11) = sgnDeriv(m_chain->getQd()(i1), eps);
                }
            }
        }
        void calcSlotineLiRegressorDqdd(const aligned_vector<Jacobian<T> > &jacobians, aligned_vector<Regressor<T> > &regressorsDq, const T eps = T(1e-4)) {
            size_t regressorCols = regressorsDq[0].cols() / m_chain->getNumOfJoints();
            for (size_t i1 = 0; i1 < m_chain->getNumOfJoints(); i1++) {
                regressorsDq[i1].setZero();
                for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
                    momentumRegressor(jacobians[i].col(i1), mt_momentumRegressor);
                    regressorsDq[i1].block(0, regressorCols * i, m_chain->getNumOfJoints(), 10) = jacobians[i].transpose() * mt_momentumRegressor;
                }
            }
        }
    };
}

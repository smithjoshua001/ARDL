#pragma once

#include "ARDL/Dynamics/LinkRegressors.hpp"
#include "ARDL/Kinematics/ForwardKinematicsTree.hpp"
#include "ARDL/Model/Tree.hpp"
#include "ARDL/Util/Filter.hpp"
#include "ARDL/Util/Math.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"
#include "ARDL/Util/sobol.hpp"
#include "ARDL/typedefs.hpp"
namespace ARDL {
using namespace Model;
using namespace Math;
using namespace Util::Filters;
using namespace Util::Math;
using namespace Regressors;
template <typename T> class DynamicsTree {
  private:
    std::shared_ptr<Tree<T>> m_tree;

    Gravity<T> m_gravity;
    size_t m_baseParamNo;

    Regressor<T> m_projRegressor, m_projParameters;
    MatrixX<T> m_regressorProjected;

  public:
    /**
         * @brief Construct a new DynamicsTree object by copying
         *
         * @param copy DynamicsTree to copy
         * @param tree New tree to point to if neccessary
         */
    DynamicsTree(const DynamicsTree<T> &copy, std::shared_ptr<Tree<T>> tree) {
        m_tree = tree;
        m_gravity = copy.m_gravity;
    }

    /**
         * @brief Construct a new DynamicsTree object
         *
         * @param tree Robot Model (tree)
         * @param grav Global gravity vector in origin
         */
    DynamicsTree(std::shared_ptr<Tree<T>> &tree, Gravity<T> &grav) {
        this->m_tree = tree;
        this->m_gravity = grav;
    }

    /**
         * @brief Construct a new DynamicsTree object
         *
         * @param tree Robot Model (tree)
         */
    DynamicsTree(std::shared_ptr<Tree<T>> &tree) {
        this->m_tree = tree;
        this->m_gravity.setZero();
        this->m_gravity(2) = 9.81;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
         * @brief Get the tree object
         *
         * @return std::shared_ptr<tree<T>>& Pointer to robot model
         */
    std::shared_ptr<Tree<T>> &getTree() { return m_tree; }

    /**
         * @brief Set global acceleration due to gravity
         *
         * @param gravity 6D vector of global gravity
         */
    void setGravity(const Gravity<T> &gravity) { this->m_gravity = gravity; }

    /**
         * @brief Calculate the Joint Inertia matrix
         *
         * @tparam InertiaD Subtype of inertia matrix (allow for block)
         * @param jacobians Aligned vector of jacobians
         * @param M Inertia matrix
         */
    template <Frame OF = Frame::SPATIAL, typename InertiaD>
    void calcJointInertiaMatrix(const aligned_vector<Pose<T>> poses,
                                const aligned_vector<Jacobian<T>> &jacobians,
                                Eigen::MatrixBase<InertiaD> const &M) {
        size_t i = 0;
        Jacobian<T> tmp(m_tree->getNumOfJoints());
        SpatialInertia<T> tmpSpatial;
        const_cast<Eigen::MatrixBase<InertiaD> &>(M).setZero();
        for (std::shared_ptr<Link<T>> &link : m_tree->getLinksRef()) {
            if (!link->isRoot() &&
                !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                tmpSpatial = link->getSI();
                if constexpr (OF == Frame::SPATIAL) {
                    tmpSpatial.applyInverseXIX(poses[i]);
                }
                //TODO optimize for number of computations (doing whole jacobian is wasteful) kept as this for now as it's easier for a tree (possibly break into separate or some store of continuity)
                tmpSpatial.applyOut(jacobians[i], tmp);
                const_cast<Eigen::MatrixBase<InertiaD> &>(M).noalias() +=
                    jacobians[i].transpose() * tmp;
                i++;
            }
        }
    }

    /**
         * @brief Calculate the Joint Inertia matrix
         *
         * @tparam InertiaD Subtype of inertia matrix (allow for block)
         * @param jacobians Aligned vector of jacobians
         * @param M Inertia matrix
         */
    template <Frame OF = Frame::SPATIAL, typename InertiaD>
    void
    calcJointInertiaMatrixOptim(const aligned_vector<Pose<T>> poses,
                                const aligned_vector<Jacobian<T>> &jacobians,
                                Eigen::MatrixBase<InertiaD> const &M) {
        const_cast<Eigen::MatrixBase<InertiaD> &>(M).setZero();

        Jacobian<T> tmp(m_tree->getNumOfJoints());
        SpatialInertia<T> tmpSpatial;
        for (size_t i = 0; i < m_tree->getMoveableLinks().size() - 1; i++) {
            tmpSpatial = m_tree->getMoveableLinks()[i + 1]->getSI();
            if constexpr (OF == Frame::SPATIAL) {
                tmpSpatial.applyInverseXIX(poses[i + 1]);
            }
            tmp.setZero();
            tmpSpatial.applyOut(jacobians[i], tmp);
            const_cast<Eigen::MatrixBase<InertiaD> &>(M).noalias() +=
                jacobians[i].transpose() * tmp;
        }
    }

    /**
         * @brief Calculate Coriolis and centrifugal matrix
         *
         * @tparam Derived Subtype of output matrix
         * @param jacobians Aligned Vector of jacobians
         * @param jacobianDots Aligned vector of the derivative of the jacobians
         * @param vels Velocity for each link
         * @param C Output Coriolis and centrifugal matrix
         */
    template <Frame OF = Frame::SPATIAL, typename Derived>
    void calcCoriolisMatrixOptim(aligned_vector<Jacobian<T>> &jacobians,
                                 aligned_vector<Jacobian<T>> &jacobianDots,
                                 aligned_vector<Motion<T>> &vels,
                                 aligned_vector<Pose<T>> &poses,
                                 Eigen::MatrixBase<Derived> const &C) {
        const_cast<Eigen::MatrixBase<Derived> &>(C).setZero();

        SpatialInertia<T> tmpSpatial;
        MatrixX<T, 6, 6> tmp;
        Jacobian<T> tmpJac(m_tree->getNumOfJoints());
        for (size_t i = 0; i < m_tree->getMoveableLinks().size() - 1; i++) {
            if constexpr (OF == Frame::SPATIAL) {
                tmpSpatial = m_tree->getMoveableLinks()[i + 1]->getSI();
                tmpSpatial.applyInverseXIX(poses[i + 1]);
                vels[i].applyTranspose(tmpSpatial, tmp);
                tmp = (tmp.transpose() - tmp).eval();
                tmpJac.template block<6, -1>(0, 0, 6, i + 1).setZero();
                tmpSpatial.applyOut(
                    jacobianDots[i].template block<6, Dynamic>(0, 0, 6, i + 1),
                    tmpJac.template block<6, Dynamic>(0, 0, 6, i + 1));
                tmpJac.template block<6, -1>(0, 0, 6, i + 1).noalias() +=
                    tmp * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);
                const_cast<Eigen::MatrixBase<Derived> &>(C)
                    .template block<-1, -1>(0, 0, i + 1, i + 1)
                    .noalias() += jacobians[i]
                                      .template block<6, -1>(0, 0, 6, i + 1)
                                      .transpose() *
                                  tmpJac.template block<6, -1>(0, 0, 6, i + 1);
            }
        }
    }

    /**
         * @brief Calculate Gravity vector
         *
         * @tparam Derived Subtype of gravity vector
         * @param jacobians aligned vector of jacobians
         * @param poses aligned vector of Adjoint transforms
         * @param G Output gravity vector
         */
    template <Frame OF = Frame::SPATIAL, typename Derived>
    void calcGravityVectorOptim(aligned_vector<Jacobian<T>> &jacobians,
                                aligned_vector<Pose<T>> &poses,
                                Eigen::MatrixBase<Derived> const &G) {
        const_cast<Eigen::MatrixBase<Derived> &>(G).setZero();

        SpatialInertia<T> tmpSpatial;
        VectorX<T, 6> grav, grav2;
        for (size_t i = 0; i < m_tree->getMoveableLinks().size() - 1; i++) {
            tmpSpatial = m_tree->getMoveableLinksRef()[i + 1]->getSI();
            if constexpr (OF == Frame::SPATIAL) {
                // tmpSpatial.applyInverseXIX(poses[i + 1]);
                poses[i + 1].applyInverse(m_gravity, grav);
                tmpSpatial.applyOut(grav, grav2);
                poses[i + 1].applyInverseTranspose(grav2, grav);
            }
            const_cast<MatrixBase<Derived> &>(G).head(i + 1).noalias() +=
                jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                grav;
        }
    }

    template <Frame OF = Frame::SPATIAL, typename RD, typename JD, typename JD2>
    void calcSlotineLiRegressor(const Eigen::MatrixBase<JD> &qd,
                                const Eigen::MatrixBase<JD2> &qdd,
                                aligned_vector<Pose<T>> &poses,
                                aligned_vector<Motion<T>> &vels,
                                const aligned_vector<Jacobian<T>> &jacs,
                                const aligned_vector<Jacobian<T>> &jDs,
                                Eigen::MatrixBase<RD> const &out) {
        const_cast<Eigen::MatrixBase<RD> &>(out).setZero();
        if constexpr (Frame::SPATIAL == OF) {
            VectorX<T, 6> vel, tmp;
            MatrixX<T, 6, 10> linkRegressor, linkRegressor2, lr3;
            for (size_t i = 0; i < m_tree->getMoveableLinks().size() - 1; i++) {
                vel = m_gravity;
                vel += jacs[i] * qdd +
                       (vels[i].getMatrix() * jacs[i] + jDs[i]) * qd;
                // vel = jDs[i]*qd;
                poses[i + 1].applyInverse(vel, tmp);
                momentumRegressor(tmp, linkRegressor);
                tmp = jacs[i] * qd;
                poses[i + 1].applyInverse(tmp, vel);
                momentumRegressor(vel, linkRegressor2);
                poses[i + 1].applyInverseTranspose(linkRegressor2, lr3);
                // linkRegressor2 = vels[i].getMatrix().transpose() * lr3;
                vels[i].applyTransposeTo(lr3, linkRegressor2);
                poses[i + 1].applyInverseTranspose(linkRegressor, lr3);
                lr3 -= linkRegressor2;
                const_cast<Eigen::MatrixBase<RD> &>(out)
                    .template block<-1, 10>(0, 10 * i, i + 1, 10)
                    .noalias() =
                    jacs[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                    lr3;
            }
        }
    }

    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &
    getRegressorProjector() {
        return m_projRegressor;
    }

    void setRegressorProjector(
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &in) {
        m_projRegressor = in;
        m_baseParamNo = in.cols();
    }
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &
    getParameterProjector() {
        return m_projParameters;
    }
    const size_t &getNumOfBaseParams() { return m_baseParamNo; }

    void calcBaseProjection(size_t random_samples, T qddLimit = 1,
                            T threshold = T(1e-8)) {
        // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressors(random_samples * m_tree->getNumOfJoints(),
        //                                                             10 * m_tree->getNumOfJoints());
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R(
            10 * m_tree->getNumOfJoints(), 10 * m_tree->getNumOfJoints());
        ForwardKinematicsTree<T> fk(m_tree);
        aligned_vector<Pose<T>> poses(m_tree->getNumOfJoints() + 1);
        aligned_vector<Motion<T>> vels(m_tree->getNumOfJoints());
        aligned_vector<Jacobian<T>> jacobians, jacobianDots;

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressors(
            m_tree->getNumOfJoints(), 10 * m_tree->getNumOfJoints());

        ARDL::Util::init(jacobians, m_tree->getNumOfJoints());
        ARDL::Util::init(jacobianDots, m_tree->getNumOfJoints());

        VectorX<T> q, qd, qdd;
        q.resize(m_tree->getNumOfJoints());
        qd.resize(m_tree->getNumOfJoints());
        qdd.resize(m_tree->getNumOfJoints());
        R.setZero();

        regressors.setZero();
        SobolData *sobolGen = sobol_create(m_tree->getNumOfJoints() * 3);
        std::vector<std::pair<T, T>> limits(m_tree->getNumOfJoints());
        m_tree->getJointLimits(limits);
        ARDL::VectorX<T> qdLimits(m_tree->getNumOfJoints());
        m_tree->getJointVelocityLimits(qdLimits);
        double *sobolRandom =
            (double *)malloc(m_tree->getNumOfJoints() * 3 * sizeof(double));
        double *lb =
            (double *)malloc(m_tree->getNumOfJoints() * 3 * sizeof(double));
        double *ub =
            (double *)malloc(m_tree->getNumOfJoints() * 3 * sizeof(double));
        for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
            lb[i] = limits[i].first;
            ub[i] = limits[i].second;
            lb[i + m_tree->getNumOfJoints()] = -qdLimits(i);
            ub[i + m_tree->getNumOfJoints()] = qdLimits(i);
            lb[i + 2 * m_tree->getNumOfJoints()] = -qddLimit;
            ub[i + 2 * m_tree->getNumOfJoints()] = qddLimit;
        }

        // CREATE RANDOM REGRESSOR STACK
        for (size_t k = 0; k < random_samples; k++) {
            sobol_next(sobolGen, sobolRandom, lb, ub);
            for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
                q(i) = (T)sobolRandom[i];
                qd(i) = (T)sobolRandom[i + m_tree->getNumOfJoints()];
                qdd(i) = (T)sobolRandom[i + 2 * m_tree->getNumOfJoints()];
            }
            // m_tree->random();
            // qd= m_tree->getQd();
            // qdd.setRandom();
            // qdd*= qddLimit;
            m_tree->update(q, qd);
            m_tree->updateMatricesOptim();
            fk.template getPosesOptim<Frame::SPATIAL>(poses);
            fk.template getJacobians<Frame::SPATIAL>(poses, jacobians);

            fk.template getJacobians<ARDL::Frame::SPATIAL>(poses, jacobians);
            fk.getVelocities(vels, jacobians);
            fk.template getJacobianDots<ARDL::Frame::SPATIAL>(poses, vels, jacobians,
                                                     jacobianDots);
            this->calcSlotineLiRegressor<ARDL::Frame::SPATIAL>(
                qd, qdd, poses, vels, jacobians, jacobianDots, regressors);

            R += (regressors.transpose() * regressors); // / random_samples;
        }
        free(sobolRandom);
        free(lb);
        free(ub);
        sobol_destroy(sobolGen);
        Eigen::ColPivHouseholderQR<
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>
            qr(R);
        qr.setThreshold(threshold);
        qr.compute(R);

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Q(
            m_tree->getNumOfJoints() * 10, m_tree->getNumOfJoints() * 10);
        Q.setZero();
        Q = qr.householderQ();

        Eigen::Matrix<T, -1, -1> RQ(m_tree->getNumOfJoints() * 10,
                                    m_tree->getNumOfJoints() * 10);
        RQ.setZero();
        RQ = qr.matrixR();

        Eigen::Matrix<T, -1, -1> PQ(m_tree->getNumOfJoints() * 10,
                                    m_tree->getNumOfJoints() * 10);
        PQ.setZero();
        PQ = qr.colsPermutation();

        m_baseParamNo = qr.rank();
        // Pb
        m_projRegressor = PQ.block(0, 0, PQ.rows(), m_baseParamNo);

        // R1^{-1}*R2
        m_projParameters =
            RQ.block(0, 0, m_baseParamNo, m_baseParamNo).inverse() *
            RQ.block(0, m_baseParamNo, m_baseParamNo,
                     RQ.cols() - m_baseParamNo);

        m_projParameters = (m_projParameters.array().abs() > threshold)
                               .matrix()
                               .template cast<T>()
                               .cwiseProduct(m_projParameters);

        ARDL::MatrixX<T> K =
            m_projRegressor.transpose() +
            m_projParameters *
                PQ.block(0, m_baseParamNo, PQ.rows(), PQ.cols() - m_baseParamNo)
                    .transpose();
        m_projParameters = K;
        // m_projRegressor = K.transpose();
        // m_projRegressor= m_projParameters.transpose() * (m_projParameters *
        // m_projParameters.transpose()).inverse();
        // ARDL::MatrixX<T> Wr(m_baseParamNo, m_baseParamNo);
        // ARDL::MatrixX<T> We(m_baseParamNo, RQ.rows()-m_baseParamNo);
        // Wr= Q.block(0, 0, Q.rows(), m_baseParamNo) * RQ.block(0, 0, m_baseParamNo, m_baseParamNo);
        // We =
        m_regressorProjected.resize(m_tree->getNumOfJoints(), m_baseParamNo);
    }
};
} // namespace ARDL

#pragma once

#include "ARDL/Model/Tree.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
using namespace Model;
using namespace Math;
using namespace Frames;
template <typename T, Frame F = Frame::SPATIAL> class ForwardKinematicsTree {
  private:
    std::shared_ptr<Tree<T>> m_tree;

    fcl::CollisionRequest<T> m_request;
    fcl::CollisionResult<T> m_result;

    Eigen::Matrix<T, 6, 36> m_w;

    /**************TEMPORARIES**************/
    Pose<T> m_accumulatorLocal, m_accumulatorGlobal;
    Motion<T> m_bodyVelocity, m_bodyVelocity2;
    Jacobian<T> m_jacobian, m_jacobianDot;

    std::vector<std::pair<size_t, size_t>> m_collisionPairs;
    std::pair<size_t, size_t> m_pair;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //NOLINT(llvmlibc-callee-namespace)

    ForwardKinematicsTree(const ForwardKinematicsTree &copy,
                          std::shared_ptr<Tree<T>> tree) {
        m_tree = tree;
        m_request = copy.m_request;
        m_result = copy.m_result;
        m_w = copy.m_w;
        m_accumulatorLocal = copy.m_accumulatorLocal;
        m_accumulatorGlobal = copy.m_accumulatorGlobal;
        m_bodyVelocity = copy.m_bodyVelocity;
        m_bodyVelocity2 = copy.m_bodyVelocity2;
        m_jacobian = copy.m_jacobian;
        m_jacobianDot = copy.m_jacobianDot;
    }

    explicit ForwardKinematicsTree(std::shared_ptr<Tree<T>> tree) {
        this->m_tree = tree;
        m_jacobian.resize(tree->getNumOfJoints());
        m_jacobianDot.resize(tree->getNumOfJoints());
        Eigen::Matrix<T, 3, 3> iden33;
        iden33.setIdentity();
        Eigen::Matrix<T, 3, 6 * 3> ww;
        ww.setZero();
        ww.template block<1, 3>(1, 3) = iden33.template block<1, 3>(2, 0);
        ww.template block<1, 3>(2, 3) = -iden33.template block<1, 3>(1, 0);
        ww.template block<1, 3>(0, 9) = -iden33.template block<1, 3>(2, 0);
        ww.template block<1, 3>(2, 9) = iden33.template block<1, 3>(0, 0);
        ww.template block<1, 3>(0, 15) = iden33.template block<1, 3>(1, 0);
        ww.template block<1, 3>(1, 15) = -iden33.template block<1, 3>(0, 0);

        Eigen::Matrix<T, 3, 6 * 3> wv;
        wv.setZero();
        wv.template block<1, 3>(1, 0) = iden33.template block<1, 3>(2, 0);
        wv.template block<1, 3>(2, 0) = -iden33.template block<1, 3>(1, 0);
        wv.template block<1, 3>(0, 6) = -iden33.template block<1, 3>(2, 0);
        wv.template block<1, 3>(2, 6) = iden33.template block<1, 3>(0, 0);
        wv.template block<1, 3>(0, 12) = iden33.template block<1, 3>(1, 0);
        wv.template block<1, 3>(1, 12) = -iden33.template block<1, 3>(0, 0);

        m_w.setZero();
        m_w.template block<3, 18>(0, 0) = wv;
        m_w.template block<3, 18>(3, 18) = wv;
        m_w.template block<3, 18>(3, 0) = ww;
        m_collisionPairs.reserve(tree->getNumOfLinks() * tree->getNumOfLinks());
    }

    template <Frame OF = F>
    void getPosesOptim(aligned_vector<Pose<T>> &poses,
                       bool updateCollision = false) {
        if constexpr (OF == Frame::SPATIAL) {
            assert(poses.size() == m_tree->getMoveableLinks().size());
            poses[0].setIdentity();

            if (updateCollision) {
                m_tree->getMoveableLinksRef()[0]->updateCollisionOptim(
                    poses[0]);
            }
            for (size_t i = 1; i < m_tree->getMoveableLinksRef().size(); i++) {
                Link<T> *mLink = m_tree->getMoveableLinksRef()[i];
                poses[m_tree->getMoveableParentId(mLink)].applyTo(
                    ARDL_visit(mLink->getParentJoint(), getAdjointLocal()),
                    poses[i]);
                if (updateCollision) {
                    mLink->updateCollisionOptim(poses[i]);
                }
            }
        }
    }

    // template <Frame OF=F> void getPosesDqOptim(aligned_vector<aligned_vector<Pose<T>>>& posesDq){
    //     if constexpr (OF == Frame::SPATIAL){

    //     }
    // }

    template <Frame OF = F>
    void getJacobians(const aligned_vector<Pose<T>> &poses,
                      aligned_vector<Jacobian<T>> &jacs) {
        jacs.front().setZero();

        if constexpr (OF == Frame::SPATIAL) {
            Jacobian<T> jacobian(m_tree->getNumOfJoints());
            for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
                jacobian.col(i) = ARDL_visit(
                    m_tree->getMoveableLinksRef()[i + 1]->getParentJoint(),
                    getS());
            }
            poses[1].apply(jacobian.col(0), jacobian.col(0));
            jacs[0].col(0) = jacobian.col(0);
            for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
                jacs[i].template block<6, -1>(0, 0, 6, i).noalias() =
                    jacs[m_tree->getMoveableParentId(
                             m_tree->getMoveableLinksRef()[i + 1]) -
                         1]
                        .template block<6, -1>(0, 0, 6, i);
                poses[i + 1].apply(jacobian.col(i), jacobian.col(i));
                jacs[i].col(i) = jacobian.col(i);
            }
        }
    }

    template <Frame OF = F>
    void getJacobianDots(aligned_vector<Pose<T>> &poses,
                         aligned_vector<Motion<T>> &vels,
                         aligned_vector<Jacobian<T>> &jacs,
                         aligned_vector<Jacobian<T>> &jacDots) {
        jacDots.front().setZero();
        if constexpr (OF == Frame::SPATIAL) {
            Jacobian<T> jacobianD(m_tree->getNumOfJoints());
            jacDots[0].col(0) = jacs[0].col(0);
            vels[0].apply(jacDots[0].col(0), jacDots[0].col(0),
                          jacobianD.col(0));
            for (size_t i = 1; i < m_tree->getNumOfJoints(); i++) {
                jacDots[i].template block<6, -1>(0, 0, 6, i).noalias() =
                    jacDots[m_tree->getMoveableParentId(
                                m_tree->getMoveableLinksRef()[i + 1]) -
                            1]
                        .template block<6, -1>(0, 0, 6, i);
                jacDots[i].col(i) = jacs[i].col(i);
                vels[i].apply(jacDots[i].col(i), jacDots[i].col(i),
                              jacobianD.col(0));
            }
        }
    }

    void getVelocities(aligned_vector<Motion<T>> &vels,
                       aligned_vector<Jacobian<T>> &jacobians) {
        for (size_t t_j = 0; t_j < jacobians.size(); t_j++) {
            vels[t_j] = jacobians[t_j].template block<6, Eigen::Dynamic>(
                            0, 0, 6, t_j + 1) *
                        m_tree->getQd().head(t_j + 1);
        }
    }

    template <Frame OF = F>
    void getJacobiansDq(aligned_vector<Pose<T>> &poses,
                        aligned_vector<Jacobian<T>> &jacs,
                        aligned_vector<aligned_vector<Jacobian<T>>> &jacsDq) {
        if constexpr (OF == Frames::SPATIAL) {
            Jacobian<T> jacobian(m_tree->getNumOfJoints());
            ARDL::VectorX<T, 6> vec;
            Motion<T> vel;
            for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
                jacobian.col(i) = ARDL_visit(
                    m_tree->getMoveableLinksRef()[i + 1]->getParentJoint(),
                    getS());

            }

                jacsDq[0][0].setZero();
            // Jp+1 = Jp + Ad0_p S
            // Jp+1/dq = Jp/dq + Ad0_p/dq S
            //Ad0_p/dq =  adj_jk Ad0_p

            for (size_t i = 1; i < m_tree->getNumOfJoints(); i++) {
                Pose<T> pose = poses[i];
                //Ad_0p S
                poses[i + 1].apply(jacobian.col(i), vec);
                for (size_t j = 0; j < i; j++) {
                    //adj_jk
                    vel.getVelocity() = jacs[i].col(j);
                    //Jp+1 = Jp
                    jacsDq[i][j].block(0, 0, 6, i) =
                        jacsDq[m_tree->getMoveableParentId(
                                   m_tree->getMoveableLinksRef()[i + 1]) -
                               1][j]
                            .block(0, 0, 6, i);
                    //Jp+1 = adj_jk Ad0_p S
                    vel.apply(vec, jacsDq[i][j].col(i));
                }
            }
        }
    }

    template <Frame OF = F>
    void
    getJacobianDotsDq(const aligned_vector<Pose<T>> &poses,
                      aligned_vector<Motion<T>> &vels,
                      aligned_vector<Jacobian<T>> &jacDots,
                      const aligned_vector<aligned_vector<Jacobian<T>>> &jacsDq,
                      aligned_vector<aligned_vector<Jacobian<T>>> &jacDotsDq) {
        if constexpr (OF == Frames::SPATIAL) {
            // Jdp+1 = Jdp + adj_pk Ad0_p S
            // Jdp+1/dq = dJp/dq + (adj_pk Ad0_p)/dq S
            //(adj_pk Ad0_p)/dt = adj_pk/dq Ad0_p + adj_pk adj_jk Ad0_p
            // adj_pk/dq= adj_(Jdq qd)

            Jacobian<T> jacobian(m_tree->getNumOfJoints());
            ARDL::VectorX<T, 6> vec, vec2;
            Motion<T> vel, adj_dt;

            for (size_t i = 0; i < m_tree->getNumOfJoints(); i++) {
                jacobian.col(i) = ARDL_visit(
                    m_tree->getMoveableLinksRef()[i + 1]->getParentJoint(),
                    getS());
                jacDotsDq[0][i].setZero();
            }

            for (size_t i = 1; i < m_tree->getNumOfJoints(); i++) {
                Pose<T> pose = poses[i];
                //Ad_0p S
                poses[i + 1].apply(jacobian.col(i), vec);
                for (size_t j = 0; j < i; j++) {
                    adj_dt.getVelocity() = jacsDq[i][j] * m_tree.getQd();
                    // adj_pk adj_jk Ad0_p S
                    vels[i].apply(jacsDq[i][j].col(i), jacDotsDq[i][j].col(i));
                    adj_dt.apply(vec, vec2);
                    //adj_(Jdq qd) Ad0_p + adj_pk adj_jk Ad0_p
                    jacDotsDq[i][j].col(i) += vec2;
                    jacDotsDq[i][j].block(0, 0, 6, i) =
                        jacDotsDq[m_tree->getMoveableParentId(
                                   m_tree->getMoveableLinksRef()[i + 1]) -
                               1][j]
                            .block(0, 0, 6, i);

                }
            }
        }
    }
};
} // namespace ARDL

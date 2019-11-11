#pragma once

#include "ARDL/typedefs.hpp"
#include "ARDL/Model/Chain.hpp"

namespace ARDL {
    using namespace Model;
    using namespace Math;
    template <typename T> class ForwardKinematics {
    private:
        std::shared_ptr<Chain<T> > m_chain;

        fcl::CollisionRequest<T> mt_request;
        fcl::CollisionResult<T> mt_result;

        Eigen::Matrix<T, 6, 36> mt_W;

        /**************TEMPORARIES**************/
        AdjointSE3<T> mt_accumulatorLocal, mt_accumulatorGlobal;
        LieBracketSE3<T> mt_bodyVelocity, mt_bodyVelocity2;

        Jacobian<T> mt_jacobian, mt_jacobianDot;

        size_t t_i, t_j, t_j0;

        std::vector<std::pair<size_t, size_t> > collisionPairs;
        std::pair<size_t, size_t> t_pair;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        ForwardKinematics(const ForwardKinematics &copy, std::shared_ptr<Chain<T> > chain) {
            m_chain = chain;
            mt_request = copy.mt_request;
            mt_result = copy.mt_result;
            mt_W = copy.mt_W;
            mt_accumulatorLocal = copy.mt_accumulatorLocal;
            mt_accumulatorGlobal = copy.mt_accumulatorGlobal;
            mt_bodyVelocity = copy.mt_bodyVelocity;
            mt_bodyVelocity2 = copy.mt_bodyVelocity2;
            mt_jacobian = copy.mt_jacobian;
            mt_jacobianDot = copy.mt_jacobianDot;
            t_i = copy.t_i;
            t_j = copy.t_j;
            t_j0 = copy.t_j0;
        }

        ForwardKinematics(std::shared_ptr<Chain<T> > chain) {
            this->m_chain = chain;
            mt_jacobian.resize(6, chain->getNumOfJoints());
            mt_jacobianDot.resize(6, chain->getNumOfJoints());
            Eigen::Matrix<T, 3, 3> iden33;
            iden33.setIdentity();
            Eigen::Matrix<T, 3, 6 * 3> Ww;
            Ww.setZero();
            Ww.template block<1, 3>(1, 3) = iden33.template block<1, 3>(2, 0);
            Ww.template block<1, 3>(2, 3) = -iden33.template block<1, 3>(1, 0);
            Ww.template block<1, 3>(0, 9) = -iden33.template block<1, 3>(2, 0);
            Ww.template block<1, 3>(2, 9) = iden33.template block<1, 3>(0, 0);
            Ww.template block<1, 3>(0, 15) = iden33.template block<1, 3>(1, 0);
            Ww.template block<1, 3>(1, 15) = -iden33.template block<1, 3>(0, 0);

            Eigen::Matrix<T, 3, 6 * 3> Wv;
            Wv.setZero();
            Wv.template block<1, 3>(1, 0) = iden33.template block<1, 3>(2, 0);
            Wv.template block<1, 3>(2, 0) = -iden33.template block<1, 3>(1, 0);
            Wv.template block<1, 3>(0, 6) = -iden33.template block<1, 3>(2, 0);
            Wv.template block<1, 3>(2, 6) = iden33.template block<1, 3>(0, 0);
            Wv.template block<1, 3>(0, 12) = iden33.template block<1, 3>(1, 0);
            Wv.template block<1, 3>(1, 12) = -iden33.template block<1, 3>(0, 0);

            mt_W.setZero();
            mt_W.template block<3, 18>(0, 0) = Wv;
            mt_W.template block<3, 18>(3, 18) = Wv;
            mt_W.template block<3, 18>(3, 0) = Ww;
            collisionPairs.reserve(chain->getNumOfLinks() * chain->getNumOfLinks());
        }

        void getAdjoints(aligned_vector<AdjointSE3<T> > &ATs, bool update_collision = false) {
            t_j = 0;
            ATs[0].setIdentity();
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                size_t index = std::min(t_j, ATs.size() - 1);

                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    link->getParentJoint()->getAdjointLocal().apply(ATs[index], ATs[index]);
                    ATs[std::min(t_j + 1, ATs.size() - 1)] = ATs[index];
                    t_j++;
                } else if (!link->isRoot()) {
                    link->getParentJoint()->getAdjointLocal().apply(ATs[index], ATs[index]);
                }
                if (update_collision) {
                    link->updateCollision(ATs[index]);
                }
            }
            assert(t_j == m_chain->getNumOfJoints());
        }

        void getLinkAdjoints(aligned_vector<AdjointSE3<T> > &ATs, bool update_collision = false) {
            t_j = 0;
            ATs[0].setIdentity();
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                size_t index = std::min(t_j, ATs.size() - 1);
                // LOG_DEBUG_LEVEL5("INDEX: {}, J:{}, SIZE: {}, LINK: {}", index, t_j, ATs.size(), link->getName());

                if (!link->isRoot()) {
                    // LOG_DEBUG_LEVEL5("FORWARD APPLY TRANSFORM AND GENERATE NEXT ONE {} \n \n {} \n\n", link->getParentJoint()->getAdjointLocal(), link->getParentJoint()->getOriginTransform());
                    // LOG_DEBUG_LEVEL5("BEFORE: {}", ATs[index]);
                    link->getParentJoint()->getAdjointLocal().apply(ATs[index], ATs[index]);
                    // LOG_DEBUG_LEVEL5("AFTER: {}", ATs[index]);
                    ATs[std::min(t_j + 1, ATs.size() - 1)] = ATs[index];
                    t_j++;
                } else {
                    ATs[std::min(t_j + 1, ATs.size() - 1)] = ATs[index];
                    t_j++;
                }
                if (update_collision) {
                    // LOG_DEBUG_LEVEL5("UPDATE COLLISION");
                    link->updateCollision(ATs[index]);
                }
            }
            assert(t_j == m_chain->getNumOfLinks());
        }

        void getInverseAdjoints(aligned_vector<AdjointSE3<T> > &ATs, bool update_collision = false) {
            t_j = 0;
            ATs[0].setIdentity();
            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                size_t index = std::min(t_j, ATs.size() - 1);
                LOG_DEBUG_LEVEL5("INDEX: {}, J:{}, SIZE: {}, LINK: {}", index, t_j, ATs.size(), link->getName());
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    link->getParentJoint()->getAdjointLocal().applyInverseTo(ATs[index], ATs[index]);
                    ATs[std::min(t_j + 1, ATs.size() - 1)] = ATs[index];
                    t_j++;
                } else if (!link->isRoot()) {
                    link->getParentJoint()->getAdjointLocal().applyInverseTo(ATs[index], ATs[index]);
                }
            }
            assert(t_j == m_chain->getNumOfJoints());
        }

        void getLieBrackets(aligned_vector<LieBracketSE3<T> > &adjs, aligned_vector<Jacobian<T> > &jacobians) {
            t_j = 0;
            for (Jacobian<T> &jacobian: jacobians) {
                adjs[t_j].setVelocity(jacobian.block(0, 0, 6, t_j + 1) * m_chain->getQd().head(t_j + 1));
                t_j++;
            }
        }

        void getBodyJacobian(aligned_vector<Jacobian<T> > &jacobian, aligned_vector<Jacobian<T> > &jacobianDot) {
            mt_accumulatorLocal.setIdentity();
            t_j = 0;
            jacobian[t_j].setZero();
            jacobianDot[t_j].setZero();

            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    size_t index = std::max((int)t_j - 1, 0);
                    jacobian[t_j].setZero();
                    for (t_i = 0; t_i < t_j; t_i++) {
                        link->getParentJoint()->getAdjointLocal().applyInverse(jacobian[index].template block<6, 1>(0, t_i), jacobian[t_j].template block<6, 1>(0, t_i));
                        mt_accumulatorLocal.applyInverse(jacobian[t_j].template block<6, 1>(0, t_i), jacobian[t_j].template block<6, 1>(0, t_i));
                    }

                    jacobianDot[t_j].setZero();
                    for (t_i = 0; t_i < t_j; t_i++) {
                        link->getParentJoint()->getAdjointLocal().applyInverse(jacobianDot[index].template block<6, 1>(0, t_i), jacobianDot[t_j].template block<6, 1>(0, t_i));
                        mt_accumulatorLocal.applyInverse(jacobianDot[t_j].template block<6, 1>(0, t_i), jacobianDot[t_j].template block<6, 1>(0, t_i));
                    }

                    mt_bodyVelocity.getVelocity().setZero();
                    mt_accumulatorLocal.applyInverse(link->getParentJoint()->getVelocityVector(), mt_bodyVelocity.getVelocity());
                    mt_bodyVelocity.calcMatrix();
                    jacobianDot[t_j].noalias() -= (mt_bodyVelocity.getMatrix() * jacobian[t_j]);
                    mt_accumulatorLocal.apply(link->getParentJoint()->getS(), jacobian[t_j].template block<6, 1>(0, t_j));

                    mt_accumulatorLocal.setIdentity();

                    t_j++;
                } else if (!link->isRoot() && link->getParentJoint()->isFixed()) {
                    link->getParentJoint()->getAdjointLocal().apply(mt_accumulatorLocal, mt_accumulatorLocal);
                }
            }
            assert(t_j == m_chain->getNumOfJoints());

            for (t_i = 0; t_i < t_j; t_i++) {
                mt_accumulatorLocal.applyInverse(jacobian[t_j - 1].template block<6, 1>(0, t_i), jacobian[t_j - 1].template block<6, 1>(0, t_i));
                mt_accumulatorLocal.applyInverse(jacobianDot[t_j - 1].template block<6, 1>(0, t_i), jacobianDot[t_j - 1].template block<6, 1>(0, t_i));
            }
        }

        void convertBodyToMixedJacobian(const AdjointSE3<T> &adjointTransform, const Jacobian<T> &b_jacobian, Jacobian<T> &m_jacobian) {
            mt_accumulatorLocal = adjointTransform;
            mt_accumulatorLocal.getPRef().setZero();

            m_jacobian = mt_accumulatorLocal.getMatrix() * b_jacobian;
        }

        void convertBodyToMixedJacobianDot(const AdjointSE3<T> &adjointTransform, const Jacobian<T> &m_jacobian, const Jacobian<T> &b_jacobianDot, Jacobian<T> &m_jacobianDot) {
            mt_accumulatorLocal = adjointTransform;
            mt_accumulatorLocal.getPRef().setZero();

            mt_bodyVelocity.getVelocity().template head<3>().setZero();
            mt_bodyVelocity.getVelocity().template tail<3>() = m_jacobian.template block(3, 0, 3, m_chain->getNumOfJoints()) * m_chain->getQd();
            mt_bodyVelocity.calcMatrix();

            m_jacobianDot = mt_accumulatorLocal.getMatrix() * b_jacobianDot + mt_bodyVelocity.getMatrix() * m_jacobian;
        }

        template <int k> void convertBodyToMixedJacobiansDq(const aligned_vector<AdjointSE3<T> > &adjointTransforms, const Jacobian<T> &m_jacobian, const aligned_vector<Jacobian<T> > &b_jacobiansDQ, aligned_vector<Jacobian<T> > &m_jacobiansDQ) {
            for (size_t index = 0; index < m_chain->getNumOfJoints(); index++) {
                mt_accumulatorLocal = adjointTransforms.back();
                mt_accumulatorLocal.getPRef().setZero();
                mt_bodyVelocity.getVelocity() = m_jacobian.col(index);

                mt_bodyVelocity.getVelocity().template head<3>().setZero();
                mt_bodyVelocity.calcMatrix();
                m_jacobiansDQ[index] = mt_accumulatorLocal.getMatrix() * b_jacobiansDQ[index] + mt_bodyVelocity.getMatrix() * m_jacobian;
            }
        }

        template <int k> void convertBodyToMixedJacobiansDotDq(const aligned_vector<AdjointSE3<T> > &adjointTransforms,
                                                               const Jacobian<T> &b_jacobian,
                                                               const aligned_vector<Jacobian<T> > &b_jacobiansDQ,
                                                               const Jacobian<T> &b_jacobianDot,
                                                               const aligned_vector<Jacobian<T> > &b_jacobiansDotDQ,
                                                               aligned_vector<Jacobian<T> > &m_jacobiansDotDQ) {
            for (size_t index = 0; index < m_chain->getNumOfJoints(); index++) {
                mt_accumulatorLocal = adjointTransforms.back();
                mt_bodyVelocity.getVelocity() = b_jacobian.col(index);
                mt_bodyVelocity.getVelocity().template head<3>().setZero();
                mt_bodyVelocity.calcMatrix();
                mt_accumulatorLocal.getPRef().setZero();

                m_jacobiansDotDQ[index] = mt_accumulatorLocal.getMatrix() * b_jacobiansDotDQ[index] + mt_accumulatorLocal.getMatrix() * mt_bodyVelocity.getMatrix() * b_jacobianDot;

                mt_bodyVelocity2.getVelocity().template head<3>().setZero();
                mt_bodyVelocity2.getVelocity().template tail<3>() = b_jacobian.template block(3, 0, 3, m_chain->getNumOfJoints()) * m_chain->getQd();
                mt_bodyVelocity2.calcMatrix();

                m_jacobiansDotDQ[index] += mt_accumulatorLocal.getMatrix() * mt_bodyVelocity2.getMatrix() * b_jacobiansDQ[index] + mt_accumulatorLocal.getMatrix() * mt_bodyVelocity.getMatrix() * mt_bodyVelocity2.getMatrix() * b_jacobian;

                mt_bodyVelocity2.getVelocity().template head<3>().setZero();
                mt_bodyVelocity2.getVelocity().template tail<3>() = b_jacobiansDQ[index].template block(3, 0, 3, m_chain->getNumOfJoints()) * m_chain->getQd();
                mt_bodyVelocity2.calcMatrix();

                m_jacobiansDotDQ[index] += mt_accumulatorLocal.getMatrix() * mt_bodyVelocity2.getMatrix() * b_jacobian;
            }
        }

        template <int k> void getBodyJacobianEEStateDerivative(const aligned_vector<Jacobian<T> > &body_jacobian,
                                                               const aligned_vector<Jacobian<T> > &body_jacobianDot,
                                                               aligned_vector<Jacobian<T> > &body_jacobianDq,
                                                               aligned_vector<Jacobian<T> > &body_jacobianDotDq) {
            mt_accumulatorLocal.setIdentity();
            t_j = 0;
            for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                body_jacobianDq[t_j0].setZero();
                body_jacobianDotDq[t_j0].setZero();
            }
            mt_jacobian.setZero();
            mt_jacobianDot.setZero();

            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    size_t index = std::max((int)t_j - 1, 0);
                    link->getParentJoint()->getAdjointLocal().apply(mt_accumulatorLocal, mt_accumulatorLocal);

                    for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                        for (t_i = 0; t_i < t_j; t_i++) {
                            mt_accumulatorLocal.applyInverse(body_jacobianDq[t_j0].template block<6, 1>(0, t_i), body_jacobianDq[t_j0].template block<6, 1>(0, t_i));
                            mt_accumulatorLocal.applyInverse(body_jacobianDotDq[t_j0].template block<6, 1>(0, t_i), body_jacobianDotDq[t_j0].template block<6, 1>(0, t_i));
                        }
                    }
                    if (t_j - 1 >= 0) {
                        for (t_i = 0; t_i < t_j; t_i++) {
                            mt_accumulatorLocal.applyInverse(body_jacobian[index].template block<6, 1>(0, t_i), mt_jacobian.template block<6, 1>(0, t_i));
                            mt_accumulatorLocal.applyInverse(body_jacobianDot[index].template block<6, 1>(0, t_i), mt_jacobianDot.template block<6, 1>(0, t_i));
                        }
                    }

                    //create adjunct
                    mt_bodyVelocity.setVelocity(m_chain->getJoint(t_j)->getS());

                    body_jacobianDq[t_j] -= mt_bodyVelocity.getMatrix() * mt_jacobian;

                    body_jacobianDotDq[t_j] -= mt_bodyVelocity.getMatrix() * mt_jacobianDot;

                    for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                        body_jacobianDotDq[t_j0] -= link->getParentJoint()->getAdjPK() * body_jacobianDq[t_j0];
                    }

                    mt_accumulatorLocal.setIdentity();

                    t_j++;
                } else if (!link->isRoot() && link->getParentJoint()->isFixed()) {
                    link->getParentJoint()->getAdjointLocal().apply(mt_accumulatorLocal, mt_accumulatorLocal);
                }
            }
            assert(t_j == m_chain->getNumOfJoints());
            for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                for (t_i = 0; t_i < t_j; t_i++) {
                    mt_accumulatorLocal.applyInverse(body_jacobianDq[t_j0].template block<6, 1>(0, t_i), body_jacobianDq[t_j0].template block<6, 1>(0, t_i));
                    mt_accumulatorLocal.applyInverse(body_jacobianDotDq[t_j0].template block<6, 1>(0, t_i), body_jacobianDotDq[t_j0].template block<6, 1>(0, t_i));
                }
            }
        }
        template <int k> void getBodyJacobianStateDerivatives(const aligned_vector<Jacobian<T> > &body_jacobian,
                                                              const aligned_vector<Jacobian<T> > &body_jacobianDot,
                                                              aligned_vector<aligned_vector<Jacobian<T> > > &body_jacobianDq,
                                                              aligned_vector<aligned_vector<Jacobian<T> > > &body_jacobianDotDq) {
            mt_accumulatorLocal.setIdentity();
            t_j = 0;
            for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                for (t_i = 0; t_i < m_chain->getNumOfJoints(); t_i++) {
                    body_jacobianDq[t_j0][t_i].setZero();
                    body_jacobianDotDq[t_j0][t_i].setZero();
                }
            }
            mt_jacobian.setZero();
            mt_jacobianDot.setZero();

            for (std::shared_ptr<Link<T> > &link: m_chain->getLinksRef()) {
                if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                    size_t index = std::max((int)t_j - 1, 0);
                    link->getParentJoint()->getAdjointLocal().apply(mt_accumulatorLocal, mt_accumulatorLocal);

                    for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                        for (t_i = 0; t_i < t_j; t_i++) {
                            mt_accumulatorLocal.applyInverse(body_jacobianDq[index][t_j0].template block<6, 1>(0, t_i), body_jacobianDq[t_j][t_j0].template block<6, 1>(0, t_i));
                            mt_accumulatorLocal.applyInverse(body_jacobianDotDq[index][t_j0].template block<6, 1>(0, t_i), body_jacobianDotDq[t_j][t_j0].template block<6, 1>(0, t_i));
                        }
                    }
                    if (t_j - 1 >= 0) {
                        for (t_i = 0; t_i < t_j; t_i++) {
                            mt_accumulatorLocal.applyInverse(body_jacobian[index].template block<6, 1>(0, t_i), mt_jacobian.template block<6, 1>(0, t_i));
                            mt_accumulatorLocal.applyInverse(body_jacobianDot[index].template block<6, 1>(0, t_i), mt_jacobianDot.template block<6, 1>(0, t_i));
                        }
                    }

                    //create adjunct
                    mt_bodyVelocity.setVelocity(m_chain->getJoint(t_j)->getS());

                    body_jacobianDq[t_j][t_j] -= mt_bodyVelocity.getMatrix() * mt_jacobian;

                    body_jacobianDotDq[t_j][t_j] -= mt_bodyVelocity.getMatrix() * mt_jacobianDot;

                    for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                        body_jacobianDotDq[t_j][t_j0] -= link->getParentJoint()->getAdjPK() * body_jacobianDq[t_j][t_j0];
                    }

                    mt_accumulatorLocal.setIdentity();

                    t_j++;
                } else if (!link->isRoot() && link->getParentJoint()->isFixed()) {
                    link->getParentJoint()->getAdjointLocal().apply(mt_accumulatorLocal, mt_accumulatorLocal);
                }
            }
            assert(t_j == m_chain->getNumOfJoints());
            for (t_j0 = 0; t_j0 < m_chain->getNumOfJoints(); t_j0++) {
                for (t_i = 0; t_i < t_j; t_i++) {
                    mt_accumulatorLocal.applyInverse(body_jacobianDq[t_j - 1][t_j0].template block<6, 1>(0, t_i), body_jacobianDq[t_j - 1][t_j0].template block<6, 1>(0, t_i));
                    mt_accumulatorLocal.applyInverse(body_jacobianDotDq[t_j - 1][t_j0].template block<6, 1>(0, t_i), body_jacobianDotDq[t_j - 1][t_j0].template block<6, 1>(0, t_i));
                }
            }
        }

        size_t number_of_collision_checks() {
            size_t collision_no = 0;
            for (size_t i = 0; i < m_chain->getLinks().size(); i++) {
                for (t_j = i + 2; t_j < m_chain->getLinks().size(); t_j++) {
                    if (m_chain->getLinks().data()[t_j]->hasCollision()) {
                        collision_no++;
                    }
                }
            }
            return collision_no;
        }

        bool collision() {
            int collision_no = 0;
            for (size_t i = 0; i < m_chain->getLinks().size(); i++) {
                fcl::CollisionObject<T> *obj1 = m_chain->getLinks().data()[i]->getCollisionObjectPtr().get();
                for (t_j = i + 2; t_j < m_chain->getLinks().size(); t_j++) {  //5 4 3 2 1
                    if (m_chain->getLinks().data()[t_j]->hasCollision()) {
                        fcl::CollisionObject<T> *obj2 = m_chain->getLinks().data()[t_j]->getCollisionObjectPtr().get();
                        collide(obj1, obj2, mt_request, mt_result);
                        if (mt_result.isCollision()) {
                            mt_result.clear();
                            return true;
                        }
                        collision_no++;
                    }
                }
            }
            return false;
        }
        int collision_number() {
            int collision_no = 0;
            for (size_t i = 0; i < m_chain->getLinks().size(); i++) {
                fcl::CollisionObject<T> *obj1 = m_chain->getLinks().data()[i]->getCollisionObjectPtr().get();
                for (t_j = i + 2; t_j < m_chain->getLinks().size(); t_j++) {  //5 4 3 2 1
                    if (m_chain->getLinks().data()[t_j]->hasCollision()) {
                        fcl::CollisionObject<T> *obj2 = m_chain->getLinks().data()[t_j]->getCollisionObjectPtr().get();
                        collide(obj1, obj2, mt_request, mt_result);
                        if (mt_result.isCollision()) {
                            mt_result.clear();
                            collision_no++;
                        }
                    }
                }
            }
            return collision_no;
        }

        const std::vector<std::pair<size_t, size_t> > &getCollisionPairs() {
            collisionPairs.clear();
            for (size_t i = 0; i < m_chain->getLinks().size(); i++) {
                fcl::CollisionObject<T> *obj1 = m_chain->getLinks().data()[i]->getCollisionObjectPtr().get();
                for (t_j = i + 2; t_j < m_chain->getLinks().size(); t_j++) {  //5 4 3 2 1
                    if (m_chain->getLinks().data()[t_j]->hasCollision()) {
                        fcl::CollisionObject<T> *obj2 = m_chain->getLinks().data()[t_j]->getCollisionObjectPtr().get();
                        collide(obj1, obj2, mt_request, mt_result);
                        if (mt_result.isCollision()) {
                            t_pair.first = i;
                            t_pair.second = t_j;
                            collisionPairs.push_back(t_pair);
                            mt_result.clear();
                        }
                    }
                }
            }
            return collisionPairs;
        }
    };
} // namespace DL

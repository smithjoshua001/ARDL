#pragma once

#include "ARDL/Model/Chain.hpp"
#include "ARDL/typedefs.hpp"

namespace ARDL {
using namespace Model;
using namespace Math;
namespace Frames {
enum Frame { BODY, MIXED, SPATIAL };
}
using namespace Frames;
template <typename T, Frame F = Frame::BODY> class ForwardKinematics {
private:
  std::shared_ptr<Chain<T>> m_chain;

  fcl::CollisionRequest<T> mt_request;
  fcl::CollisionResult<T> mt_result;

  Eigen::Matrix<T, 6, 36> mt_W;

  /**************TEMPORARIES**************/
  Pose<T> mt_accumulatorLocal, mt_accumulatorGlobal;
  Motion<T> mt_bodyVelocity, mt_bodyVelocity2;

  Jacobian<T> mt_jacobian, mt_jacobianDot;

  size_t t_i, t_j, t_j0;

  std::vector<std::pair<size_t, size_t>> collisionPairs;
  std::pair<size_t, size_t> t_pair;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ForwardKinematics(const ForwardKinematics &copy,
                    std::shared_ptr<Chain<T>> chain) {
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

  ForwardKinematics(std::shared_ptr<Chain<T>> chain) {
    this->m_chain = chain;
    mt_jacobian.resize(chain->getNumOfJoints());
    mt_jacobianDot.resize(chain->getNumOfJoints());
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

  void getAdjoints(aligned_vector<Pose<T>> &ATs,
                   bool update_collision = false) {
    size_t t_j = 0;
    Pose<T> &last = ATs.back();
    last.setIdentity();

    for (Link<T> &link : m_chain->getLinksRef()) {
      if (!link.isRoot() && !ARDL_visit(link.getParentJoint(), isFixed())) {
        size_t index = std::min(t_j, ATs.size() - 1);
        last.applyTo(ARDL_visit(link.getParentJoint(), getAdjointLocal()));

        ATs[index] = last;
        t_j++;
      } else if (!link.isRoot()) {
        last.applyTo(ARDL_visit(link.getParentJoint(), getAdjointLocal()));
      }
      if (update_collision) {
        link.updateCollision(last);
      }
    }
  }

  void getBodyAdjoints(aligned_vector<Pose<T>> &ATs,
                       bool update_collision = false) {
    int t_j = ATs.size() - 1;
    Pose<T> &last = ATs.front();
    last.setIdentity();
    for (auto link = m_chain->getLinksRef().rbegin();
         link != m_chain->getLinksRef().rend(); link++) {
      if (update_collision) {
        link->updateCollision(last);
      }
      if (!link->isRoot() && !ARDL_visit(link->getParentJoint(), isFixed())) {
        size_t index = std::max(t_j, 0);
        ATs[index] = last;
        ARDL_visit(link->getParentJoint(), getAdjointLocal())
            .applyInverse(last, last);
        t_j--;
      } else if (!link->isRoot()) {
        ARDL_visit(link->getParentJoint(), getAdjointLocal())
            .applyInverse(last, last);
      }
    }
  }

  void getAdjointsOptim(aligned_vector<Pose<T>> &ATs,
                        bool update_collision = false) {
    Pose<T> &last = ATs.back();
    last.setIdentity();
    ATs[0] = last;
    if (update_collision) {
      m_chain->getMoveableLinksRef()[0]->updateCollisionOptim(last);
    }
    for (size_t i = 1; i < m_chain->getMoveableLinksRef().size(); i++) {
      last.applyTo(
          ARDL_visit(m_chain->getMoveableLinksRef()[i]->getParentJoint(),
                     getAdjointLocal()));
      ATs[i] = last;
      if (update_collision) {
        m_chain->getMoveableLinksRef()[i]->updateCollisionOptim(last);
      }
    }
  }

  void getEEAdjoint(aligned_vector<Pose<T>> &ATs, Pose<T> &ee) {
    if (ARDL_visit(m_chain->getEELinkRef().getParentJoint(), isFixed())) {
      ATs.back().applyTo(ARDL_visit(m_chain->getEELinkRef().getParentJoint(),
                                    getAdjointLocal()),
                         ee);
    } else {
      ee = ATs.back();
    }
  }

    void getEEAdjoint(Pose<T> &last, Pose<T> &ee) {
    if (ARDL_visit(m_chain->getEELinkRef().getParentJoint(), isFixed())) {
      last.applyTo(ARDL_visit(m_chain->getEELinkRef().getParentJoint(),
                                    getAdjointLocal()),
                         ee);
    } else {
      ee = last;
    }
  }

  void getBodyAdjointsOptim(aligned_vector<Pose<T>> &ATs,
                            bool update_collision = false) {
    Pose<T> &last = ATs.front();
    last.setIdentity();
    for (size_t i = ATs.size() - 1; i > 0; i--) {
      ATs[i] = last;

      if (update_collision) {
        m_chain->getMoveableLinksRef()[i]->updateCollisionOptim(last);
      }
      ARDL_visit(m_chain->getMoveableLinksRef()[i]->getParentJoint(),
                 getAdjointLocal())
          .applyInverse(last, last);
    }
    if (update_collision) {
      m_chain->getMoveableLinksRef()[0]->updateCollisionOptim(last);
    }
    // ARDL_visit(m_chain->getMoveableLinksRef()[0]->getParentJoint(),
    // getAdjointLocal())
    //     .applyInverse(last, last);
    ATs[0] = last;
  }
  void getMixedAdjoints(aligned_vector<Pose<T>> &ATs,
                        bool update_collision = false) {
    size_t t_j = 0;
    Pose<T> &last = ATs.back();
    last.setIdentity();

    for (Link<T> &link : m_chain->getLinksRef()) {
      if (!link.isRoot() && !ARDL_visit(*(link.getParentJoint()), isFixed())) {
        size_t index = std::min(t_j, ATs.size() - 1);
        last.applyTo(ARDL_visit(*(link.getParentJoint()), getAdjointLocal()));

        ATs[index] = last;
        t_j++;
      } else if (!link.isRoot()) {
        last.applyTo(ARDL_visit(*(link.getParentJoint()), getAdjointLocal()));
      }
      if (update_collision) {
        link.updateCollision(last);
      }
    }
    for (size_t i = 0; i < ATs.size(); i++) {
      ATs[i].getPRef() -= ATs.back().getP();
    }
  }

  template <Frame OF = F>
  void getEEJacobian(aligned_vector<Jacobian<T>> &jacs, Jacobian<T> &ee) {

    if constexpr (OF == Frame::BODY) {
      if (ARDL_visit(m_chain->getEELinkRef().getParentJoint(), isFixed())) {
        ARDL_visit(m_chain->getEELinkRef().getParentJoint(), getAdjointLocal())
            .applyInverseMat(jacs.back(), ee);
        // ATs.back().applyTo(ARDL_visit(m_chain->getEELinkRef().getParentJoint(),
        // getAdjointLocal()), ee);
      } else {
        ee = jacs.back();
      }
    } else {
      ee = jacs.back();
    }
  }

  template <Frame OF = F>
  void getJacobians(const aligned_vector<Pose<T>> &Ads,
                    aligned_vector<Jacobian<T>> &jacs) {
    jacs.front().setZero();

    if constexpr (OF == Frame::BODY) {
      jacs[0].col(0).noalias() = ARDL_visit(
          m_chain->getMoveableLinksRef()[1]->getParentJoint(), getS());
      size_t dof = m_chain->getNumOfJoints();
      Jacobian<T> mt_jacobian(dof), mt_jac2(dof);
      mt_jacobian.col(0) = jacs[0].col(0);
      for (size_t i = 1; i < dof; i++) {
        ARDL_visit(m_chain->getMoveableLinksRef()[i + 1]->getParentJoint(),
                   getAdjointLocal())
            .applyInverseMat(
                jacs[i - 1].template block<6, Eigen::Dynamic>(0, 0, 6, i),
                jacs[i].template block<6, Eigen::Dynamic>(0, 0, 6, i));
        jacs[i].col(i).noalias() = ARDL_visit(
            m_chain->getMoveableLinksRef()[i + 1]->getParentJoint(), getS());
      }

    } else if constexpr (OF == Frame::SPATIAL) {
      Jacobian<T> mt_jacobian(m_chain->getNumOfJoints());
      for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
        mt_jacobian.col(i) = ARDL_visit(
            m_chain->getMoveableLinksRef()[i + 1]->getParentJoint(), getS());
      }
      for (size_t i = 0; i < m_chain->getNumOfJoints(); i++) {
        jacs[i].template block<6, -1>(0, 0, 6, i).noalias() =
            mt_jacobian.template block<6, -1>(0, 0, 6, i);
        Ads[i + 1].apply(mt_jacobian.col(i), mt_jacobian.col(i));
        jacs[i].col(i) = mt_jacobian.col(i);
      }
    }
  }

  template <Frame OF = F>
  void getJacobianDots(aligned_vector<Pose<T>> &Ads,
                       aligned_vector<Motion<T>> &adjs,
                       aligned_vector<Jacobian<T>> &jacs,
                       aligned_vector<Jacobian<T>> &jacDots) {
    jacDots.front().setZero();
    if constexpr (OF == Frame::SPATIAL) {
      jacDots[0].template col(0) = jacs[0].template col(0);
      adjs[0].apply(jacDots[0].template col(0), jacDots[0].template col(0),
                    mt_jacobianDot.template col(0));
      for (size_t i = 1; i < m_chain->getNumOfJoints(); i++) {
        jacDots[i].template block<6, -1>(0, 0, 6, i).noalias() =
            jacDots[i - 1].template block<6, -1>(0, 0, 6, i);
        jacDots[i].template col(i) = jacs[i].template col(i);
        adjs[i].apply(jacDots[i].template col(i), jacDots[i].template col(i),
                      mt_jacobianDot.template col(0));
      }
    } else if constexpr (OF == Frame::BODY) {
      // std::shared_ptr<Link<T>> link= m_chain->getMoveableLinksRef()[0];
      jacDots[0].setZero();
      size_t dof = m_chain->getNumOfJoints();
      Jacobian<T> mt_jacobianDot(dof);
      mt_jacobianDot.setZero();
      // Pose<T> mt_accumulatorLocal;
      // mt_accumulatorLocal.setIdentity();
      Motion<T> mt_bodyVelocity;
      for (size_t i = 1; i < dof; i++) {
        ARDL_visit(m_chain->getMoveableLinksRef()[i + 1]->getParentJoint(),
                   getAdjointLocal())
            .applyInverseMat(
                jacDots[i - 1].template block<6, Eigen::Dynamic>(0, 0, 6, i),
                jacDots[i].template block<6, Eigen::Dynamic>(0, 0, 6, i));

        mt_bodyVelocity.getVelocity() =
            ARDL_visit(m_chain->getMoveableLinksRef()[i + 1]->getParentJoint(),
                       getVelocityVector());

        mt_bodyVelocity.apply(jacs[i].template block<6, -1>(0, 0, 6, i),
                              mt_jacobianDot.template block<6, -1>(0, 0, 6, i));
        jacDots[i].template block<6, -1>(0, 0, 6, i).noalias() -=
            mt_jacobianDot.template block<6, -1>(0, 0, 6, i);
      }
    }
  }

  void getLieBrackets(aligned_vector<Motion<T>> &adjs,
                      aligned_vector<Jacobian<T>> &jacobians) {
    for (size_t t_j = 0; t_j < jacobians.size(); t_j++) {
      adjs[t_j] =
          jacobians[t_j].template block<6, Eigen::Dynamic>(0, 0, 6, t_j + 1) *
          m_chain->getQd().head(t_j + 1);
    }
  }

  void getEELieBracket(Motion<T> &eeadj, Jacobian<T> &eejacobian) {
    eeadj = eejacobian * m_chain->getQd();
  }

  template <Frame OF = F>
  void getJacobiansDq(const aligned_vector<Pose<T>> &Ads,
                      aligned_vector<Jacobian<T>> &jacs,
                      aligned_vector<aligned_vector<Jacobian<T>>> &jacsDq) {
    size_t dof = m_chain->getNumOfJoints();
    for (size_t i = 1; i < dof; i++) {
      Ads[i].applyInverseTo(Ads[i + 1], mt_accumulatorLocal);

      for (size_t i0 = 0; i0 < dof; i0++) {
        mt_accumulatorLocal.applyInverseMat(
            jacsDq[i0][i - 1].template block<6, -1>(0, 0, 6, i),
            jacsDq[i0][i].template block<6, -1>(0, 0, 6, i),
            mt_jacobianDot.template block<6, -1>(0, 0, 6, i));
      }
      // mt_jacobian.template block<6, Dynamic>(0, 0, 6, i)= jacs[i].template
      // block<6, -1>(0, 0, 6, i);
      mt_bodyVelocity = ARDL_visit(
          m_chain->getMoveableLinksRef()[i]->getParentJoint(), getS());
      mt_bodyVelocity.apply(jacs[i].template block<6, -1>(0, 0, 6, i),
                            mt_jacobian.template block<6, -1>(0, 0, 6, i),
                            mt_jacobianDot.template block<6, -1>(0, 0, 6, i));
      jacsDq[i][i].template block<6, Dynamic>(0, 0, 6, i) -=
          mt_jacobian.template block<6, -1>(0, 0, 6, i);
    }
  }

  template <Frame OF = F>
  void
  getJacobianDotsDq(const aligned_vector<Pose<T>> &Ads,
                    aligned_vector<Motion<T>> &adjs,
                    aligned_vector<Jacobian<T>> &jacDots,
                    const aligned_vector<aligned_vector<Jacobian<T>>> &jacsDq,
                    aligned_vector<aligned_vector<Jacobian<T>>> &jacDotsDq) {
    for (size_t i0 = 0; i0 < m_chain->getNumOfJoints(); i0++) {
      jacDotsDq[i0].front().setZero();
      typename aligned_vector<Pose<T>>::const_iterator it;
      if constexpr (OF == Frame::BODY) {
        for (size_t i = 1; i < m_chain->getNumOfJoints(); i++) {
          Link<T> *link = m_chain->getMoveableLinksRef()[i];
          Ads[i].applyInverseTo(Ads[i + 1], mt_accumulatorLocal);
          mt_accumulatorLocal.applyInverseMat(
              jacDotsDq[i0][i - 1].template block<6, Eigen::Dynamic>(0, 0, 6,
                                                                     i0),
              jacDotsDq[i0][i].template block<6, Eigen::Dynamic>(0, 0, 6, i0),
              mt_jacobianDot.template block<6, Eigen::Dynamic>(0, 0, 6, i0));
          if (i == i0) {
            mt_bodyVelocity = ARDL_visit(
                m_chain->getMoveableLinksRef()[i0]->getParentJoint(), getS());

            mt_accumulatorLocal.applyInverseMat(
                jacDots[i - 1].template block<6, -1>(0, 0, 6, i0),
                mt_jacobianDot.template block<6, -1>(0, 0, 6, i0),
                mt_jacobianDot.template block<6, -1>(0, 0, 6, i0));

            mt_bodyVelocity.apply(
                mt_jacobianDot.template block<6, -1>(0, 0, 6, i0),
                mt_jacobianDot.template block<6, -1>(0, 0, 6, i0),
                mt_jacobian.template block<6, -1>(0, 0, 6, i0));

            jacDotsDq[i0][i].template block<6, -1>(0, 0, 6, i0) -=
                mt_jacobianDot.template block<6, -1>(0, 0, 6, i0);
          }

          mt_bodyVelocity =
              ARDL_visit(link->getParentJoint(), getVelocityVector());
          mt_bodyVelocity.apply(
              jacsDq[i0][i].template block<6, -1>(0, 0, 6, i0),
              mt_jacobianDot.template block<6, -1>(0, 0, 6, i0),
              mt_jacobianDot.template block<6, -1>(0, 0, 6, i0));
          jacDotsDq[i0][i].template block<6, -1>(0, 0, 6, i0) -=
              mt_jacobianDot.template block<6, -1>(0, 0, 6, i0);
        }
      }
    }
  }

  size_t number_of_collision_checks() {
    size_t collision_no = 0;
    for (size_t i = 0; i < m_chain->getLinksRef().size(); i++) {
      for (t_j = i + 2; t_j < m_chain->getLinksRef().size(); t_j++) {
        if (m_chain->getLinksRef()[t_j].hasCollision()) {
          collision_no++;
        }
      }
    }
    return collision_no;
  }

  bool collision() {
    int collision_no = 0;
    for (size_t i = 0; i < m_chain->getLinksRef().size(); i++) {
      fcl::CollisionObject<T> *obj1 =
          m_chain->getLinksRef()[i].getCollisionObjectPtr().get();
      for (t_j = i + 2; t_j < m_chain->getLinksRef().size();
           t_j++) { // 5 4 3 2 1
        if (m_chain->getLinksRef()[t_j].hasCollision()) {
          fcl::CollisionObject<T> *obj2 =
              m_chain->getLinksRef()[t_j].getCollisionObjectPtr().get();
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
    for (size_t i = 0; i < m_chain->getLinksRef().size(); i++) {
      fcl::CollisionObject<T> *obj1 =
          m_chain->getLinksRef()[i].getCollisionObjectPtr().get();
      for (t_j = i + 2; t_j < m_chain->getLinksRef().size();
           t_j++) { // 5 4 3 2 1
        if (m_chain->getLinksRef()[t_j].hasCollision()) {
          fcl::CollisionObject<T> *obj2 =
              m_chain->getLinksRef()[t_j].getCollisionObjectPtr().get();
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

  const std::vector<std::pair<size_t, size_t>> &getCollisionPairs() {
    collisionPairs.clear();
    for (size_t i = 0; i < m_chain->getLinksRef().size(); i++) {
      fcl::CollisionObject<T> *obj1 =
          m_chain->getLinksRef()[i].getCollisionObjectPtr().get();
      for (t_j = i + 2; t_j < m_chain->getLinksRef().size();
           t_j++) { // 5 4 3 2 1
        if (m_chain->getLinksRef()[t_j].hasCollision()) {
          fcl::CollisionObject<T> *obj2 =
              m_chain->getLinksRef()[t_j].getCollisionObjectPtr().get();
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
  const std::vector<std::pair<size_t, size_t>> &getCollisionPairsOptim() {
    collisionPairs.clear();
    for (size_t i = 0; i < m_chain->getMoveableLinksRef().size(); i++) {
      fcl::CollisionObject<T> *obj1 =
          m_chain->getMoveableLinksRef()[i]->getCollisionObjectOptimPtr().get();
      for (t_j = i + 2; t_j < m_chain->getMoveableLinksRef().size();
           t_j++) { // 5 4 3 2 1
        if (m_chain->getMoveableLinksRef()[t_j]->hasCollision()) {
          fcl::CollisionObject<T> *obj2 = m_chain->getMoveableLinksRef()[t_j]
                                              ->getCollisionObjectOptimPtr()
                                              .get();
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

  bool collisionOptim() {
    int collision_no = 0;
    for (size_t i = 0; i < m_chain->getMoveableLinksRef().size(); i++) {
      fcl::CollisionObject<T> *obj1 =
          m_chain->getMoveableLinksRef()[i]->getCollisionObjectOptimPtr().get();
      for (t_j = i + 2; t_j < m_chain->getMoveableLinksRef().size();
           t_j++) { // 5 4 3 2 1
        if (m_chain->getMoveableLinksRef()[t_j]->hasCollision()) {
          fcl::CollisionObject<T> *obj2 = m_chain->getMoveableLinksRef()[t_j]
                                              ->getCollisionObjectOptimPtr()
                                              .get();
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
};
} // namespace ARDL

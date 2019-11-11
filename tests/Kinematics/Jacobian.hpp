#pragma once

#include "robotTestMain.cpp"
#include "Kinematics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include <chrono>

using namespace ARDL::Util;

// TEST_CASE("Jacobian Checking", "[adjoint][kinematics]") {
//     std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart;
//     std::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacdot;
//     std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac;

//     std::shared_ptr<Chain<double> > c;
//     std::shared_ptr<ForwardKinematics<double> > fk;

//     setupARDL(urdfModel, c, fk);

//     KDL::Chain chainKDL;
//     setupKDL(urdfModel, chainKDL, jnt_to_cart, jnt_to_jacdot, jnt_to_jac, c->getRootName(), c->getTipName());

//     std::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_vel = std::shared_ptr<KDL::ChainFkSolverVel_recursive>(new KDL::ChainFkSolverVel_recursive(chainKDL));

//     #ifdef EIGEN_DONT_VECTORIZE
//     RigidBodyDynamics::Model modelRBDL = setupRBDL(urdfModel);
//     #endif
//     Eigen::VectorXd q(c->getNumOfJoints());
//     Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints());

//     q.setZero();
//     qd.setZero();
//     qdd.setZero();

//     Eigen::MatrixXd invAdj;
//     invAdj.resize(6, 6);

//     c->updateChain(q, qd);
//     c->updateMatrices();

//     aligned_vector<AdjointSE3<double> > adjoints;
//     adjoints.resize(c->getNumOfJoints());
//     fk->getAdjoints(adjoints);

//     /******************KDL******************/
//     KDL::JntArrayVel KDL_state(c->getNumOfJoints());
//     KDL_state.q.data = q;
//     KDL_state.qdot.data = qd;
//     KDL::Frame cartPosFrame;
//     jnt_to_cart->JntToCart(KDL_state.q, cartPosFrame, chainKDL.getNrOfSegments());
//     KDL::Jacobian jac(c->getNumOfJoints());
//     jnt_to_jac->JntToJac(KDL_state.q, jac, chainKDL.getNrOfSegments());
//     KDL::Jacobian jacDot(c->getNumOfJoints());
//     jnt_to_jacdot->setRepresentation(1);
//     jnt_to_jacdot->JntToJacDot(KDL_state, jacDot, chainKDL.getNrOfSegments());

//     /******************END KDL******************/

//     aligned_vector<Jacobian<double> > jacobians, jacobianDots;
//     jacobians.resize(c->getNumOfJoints());
//     jacobianDots.resize(c->getNumOfJoints());
//     for (int i = 0; i < c->getNumOfJoints(); i++) {
//         jacobians.at(i).resize(6, c->getNumOfJoints());
//         jacobians.at(i).setZero();
//         jacobianDots.at(i).resize(6, c->getNumOfJoints());
//         jacobianDots.at(i).setZero();
//     }
//     Jacobian<double> jacDL, jacDotDL;
//     jacDL.resize(6, c->getNumOfJoints());
//     jacDotDL.resize(6, c->getNumOfJoints());

//     fk->getBodyJacobian(jacobians, jacobianDots);

//     for (int i = 0; i < c->getNumOfJoints(); i++) {
//         LOG_DEBUG_LEVEL5("DL JAC {}: \n {} \n\n", i, jacobians.at(i));
//     }
//     Eigen::Vector3d tmpP = adjoints.back().getPRef();
//     adjoints.back().getPRef().setZero();

//     fk->convertBodyToMixedJacobian(adjoints.back(), jacobians.back(), jacDL);
//     LOG_DEBUG_LEVEL5("DL JAC: \n {} \n\n", jacDL);
//     LOG_DEBUG_LEVEL5("KDL JAC: \n {} \n\n", jac.data);
//     #ifdef EIGEN_DONT_VECTORIZE
//     RigidBodyDynamics::UpdateKinematicsCustom(modelRBDL, &(q), NULL, NULL);
//     #endif
//     Eigen::Vector3d finalFixed;
//     finalFixed.setZero();
//     if (c->getLinks().back()->getParentJoint()->isFixed()) {
//         finalFixed = c->getLinks().back()->getParentJoint()->getOriginTransform().getR() * c->getLinks().back()->getParentJoint()->getOriginTransform().getP();
//     }
//     #ifdef EIGEN_DONT_VECTORIZE
//     Eigen::MatrixXd jacobianRBDL, jacobianRBDLFlipped;
//     jacobianRBDL.resize(6, c->getNumOfJoints());
//     jacobianRBDLFlipped.resize(6, c->getNumOfJoints());
//     jacobianRBDL.setZero();

//     RigidBodyDynamics::CalcBodySpatialJacobian(modelRBDL, q, c->getNumOfJoints(), jacobianRBDL, true);

//     // RigidBodyDynamics::CalcPointJacobian6D(modelRBDL, q, c->getNumOfJoints(), finalFixed, jacobianRBDL, true);

//     jacobianRBDLFlipped.block(0, 0, 3, c->getNumOfJoints()) = jacobianRBDL.block(3, 0, 3, c->getNumOfJoints());
//     jacobianRBDLFlipped.block(3, 0, 3, c->getNumOfJoints()) = jacobianRBDL.block(0, 0, 3, c->getNumOfJoints());

//     LOG_DEBUG_LEVEL5("RBDL JAC: \n {} \n\n", jacobianRBDLFlipped);

//     checkApproxMatrix(jacobianRBDLFlipped, jacDL);
//     #endif
// }

TEST_CASE("Jacobian Dot Checking", "[adjoint][kinematics]") {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacdot;
    std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac;

    std::shared_ptr<Chain<double> > c;
    std::shared_ptr<ForwardKinematics<double> > fk;

    setupARDL(urdfModel, c, fk);

    KDL::Chain chainKDL;
    setupKDL(urdfModel, chainKDL, jnt_to_cart, jnt_to_jacdot, jnt_to_jac, c->getRootName(), c->getTipName());

    std::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_vel = std::shared_ptr<KDL::ChainFkSolverVel_recursive>(new KDL::ChainFkSolverVel_recursive(chainKDL));

    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints());

    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    Eigen::MatrixXd invAdj;
    invAdj.resize(6, 6);

    aligned_vector<AdjointSE3<double> > adjoints;
    adjoints.resize(c->getNumOfJoints());
    fk->getAdjoints(adjoints);

    /******************KDL******************/
    KDL::JntArrayVel KDL_state(c->getNumOfJoints());
    KDL_state.q.data = q;
    KDL_state.qdot.data = qd;
    KDL::Jacobian jac(c->getNumOfJoints());

    KDL::Jacobian jacDot(c->getNumOfJoints());

    jnt_to_jacdot->setRepresentation(1);
    using namespace std::chrono;
    std::cout << "STEADY? " << std::chrono::steady_clock::is_steady << std::endl;
    steady_clock::time_point t1, t2;
    t1 = steady_clock::now();
    jnt_to_jac->JntToJac(KDL_state.q, jac, chainKDL.getNrOfSegments());
    jnt_to_jacdot->JntToJacDot(KDL_state, jacDot, chainKDL.getNrOfSegments());
    t2 = steady_clock::now();

    duration<double> time_span = duration_cast<duration<double> >(t2 - t1);

    std::cout << "It took KDL " << time_span.count() << " seconds.";
    std::cout << std::endl;

    /******************END KDL******************/

    aligned_vector<Jacobian<double> > jacobians, jacobianDots;
    jacobians.resize(c->getNumOfJoints());
    jacobianDots.resize(c->getNumOfJoints());
    for (int i = 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).resize(6, c->getNumOfJoints());
        jacobians.at(i).setZero();
        jacobianDots.at(i).resize(6, c->getNumOfJoints());
        jacobianDots.at(i).setZero();
    }
    Jacobian<double> jacDL, jacDotDL;
    jacDL.resize(6, c->getNumOfJoints());
    jacDotDL.resize(6, c->getNumOfJoints());
    t1 = steady_clock::now();
    c->updateChain(q, qd);
    c->updateMatrices();

    fk->getBodyJacobian(jacobians, jacobianDots);
    fk->convertBodyToMixedJacobian(adjoints.back(), jacobians.back(), jacDL);
    fk->convertBodyToMixedJacobianDot(adjoints.back(), jacDL, jacobianDots.back(), jacDotDL);
    t2 = steady_clock::now();
    time_span = duration_cast<duration<double> >(t2 - t1);

    std::cout << "It took ARDL " << time_span.count() << " seconds.";
    std::cout << std::endl;

    AdjointSE3<double> t = adjoints.back();

    LOG_DEBUG_LEVEL3("ARDL JD {}", jacobianDots.back());

    LOG_DEBUG_LEVEL3("KDL JD {}", jacDot.data);
    checkApproxMatrix(jacobianDots.back(), jacDot.data, 1e-10);

    // jnt_to_jacdot->setRepresentation(2);
    // jnt_to_jacdot->JntToJacDot(KDL_state, jacDot, chainKDL.getNrOfSegments());
    // jacDL = adjoints.back().getMatrix() * jacobians.back();
    // // fk->convertBodyToMixedJacobianDot(adjoints.back(), jacDL, jacobianDots.back(), jacDotDL);
    // LieBracketSE3<double> mt_bodyVelocity;
    // mt_bodyVelocity.getVelocity() = jacDL * qd;
    // mt_bodyVelocity.calcMatrix();

    // jacDotDL = adjoints.back().getMatrix() * jacobianDots.back() + mt_bodyVelocity.getMatrix() * jacDL;

    // LOG_DEBUG_LEVEL3("ARDL JD {}", jacDotDL);

    // LOG_DEBUG_LEVEL3("KDL JD {}", jacDot.data);

    // checkApproxMatrix(jacDotDL, jacDot.data, 1e-10);

    // jnt_to_jac->JntToJac(KDL_state.q, jac, chainKDL.getNrOfSegments());
    // jnt_to_jacdot->setRepresentation(0);
    // jnt_to_jacdot->JntToJacDot(KDL_state, jacDot, chainKDL.getNrOfSegments());

    // t1 = steady_clock::now();
    // fk->getBodyJacobian(jacobians, jacobianDots);
    // fk->convertBodyToMixedJacobian(adjoints.back(), jacobians.back(), jacDL);
    // fk->convertBodyToMixedJacobianDot(adjoints.back(), jacDL, jacobianDots.back(), jacDotDL);
    // t2 = steady_clock::now();
    // time_span = duration_cast<duration<double> >(t2 - t1);
    // std::cout << "It took me " << time_span.count() << " seconds.";
    // std::cout << std::endl;
    // LOG_DEBUG_LEVEL3("ARDL JD {}", jacDotDL);

    // LOG_DEBUG_LEVEL3("KDL JD {}", jacDot.data);

    // checkApproxMatrix(jacDotDL, jacDot.data, 1e-10);
}

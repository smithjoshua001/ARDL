#pragma once
#define EIGEN_RUNTIME_NO_MALLOC
#include "robotTestMain.cpp"
#include "Dynamics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include <chrono>

using namespace ARDL::Util;

TEST_CASE("Regressor Check", "[adjoint][kinematics]") {
    //--------------RBDL--------------------//
    RigidBodyDynamics::Model modelRBDL = setupRBDL(urdfModel);
    //----------------------------------//
    //--------------DL--------------------//
    std::shared_ptr<Chain<double> > c;
    std::shared_ptr<ForwardKinematics<double> > fk;
    std::shared_ptr<Dynamics<double> > dyn;

    setupARDL(urdfModel, c, fk, dyn);
    //----------------------------------//

    //--------------KDL--------------------//
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacdot;
    std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac;
    std::shared_ptr<KDL::ChainDynParam> kdl_dyn;

    KDL::Chain chainKDL;
    setupKDL(urdfModel, chainKDL, jnt_to_cart, jnt_to_jacdot, jnt_to_jac, kdl_dyn, c->getRootName(), c->getTipName());
    //----------------------------------//

    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints());

    q.setZero();
    qd.setZero();
    qdd.setZero();
    q(1) = 0.5;
    randomJointState(q, qd, qdd);
    //----------------------------------//

    RigidBodyDynamics::UpdateKinematicsCustom(modelRBDL, &(q), NULL, NULL);
    Eigen::MatrixXd invAdj;
    invAdj.resize(6, 6);

    KDL::JntArrayVel KDL_state(c->getNumOfJoints());
    KDL_state.q.data = q;
    KDL_state.qdot.data = qd;

    //----------------------------------//
    Eigen::VectorXd modelTorqueRBDL(c->getNumOfJoints());
    RigidBodyDynamics::InverseDynamics(modelRBDL, q, qd, qdd, modelTorqueRBDL);

    //----------------------------------//

    aligned_vector<AdjointSE3<double> > adjoints;
    adjoints.resize(c->getNumOfJoints());
    aligned_vector<LieBracketSE3<double> > adjs(c->getNumOfJoints()); aligned_vector<Jacobian<double> > jacobians, jacobianDots;
    jacobians.resize(c->getNumOfJoints());
    jacobianDots.resize(c->getNumOfJoints());
    for (int i = 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).resize(6, c->getNumOfJoints());
        jacobians.at(i).setZero();
        jacobianDots.at(i).resize(6, c->getNumOfJoints());
        jacobianDots.at(i).setZero();
    }
    Eigen::MatrixXd M(c->getNumOfJoints(), c->getNumOfJoints());
    Eigen::MatrixXd C(c->getNumOfJoints(), c->getNumOfJoints());
    Eigen::VectorXd DL_c(c->getNumOfJoints());
    Eigen::VectorXd g(c->getNumOfJoints());
    Eigen::VectorXd modelTorque(c->getNumOfJoints());
    Eigen::MatrixXd regressor(c->getNumOfJoints(), c->getNumOfJoints() * 12);
    Eigen::VectorXd params(c->getNumOfJoints() * 12);
    Eigen::VectorXd f(c->getNumOfJoints());
    using namespace std::chrono;
    steady_clock::time_point t1, t2;
    t1 = steady_clock::now();
    LOG_DEBUG_LEVEL6("TURNING OFF MALLOC 1");
    Eigen::internal::set_is_malloc_allowed(false);

    c->updateChain(q, qd);
    c->updateMatrices();

    fk->getAdjoints(adjoints);

    fk->getBodyJacobian(jacobians, jacobianDots);

    fk->getLieBrackets(adjs, jacobians);

    dyn->calcJointInertiaMatrix(jacobians, M);

    dyn->calcCoriolisMatrix(jacobians, jacobianDots, adjs, C);
    DL_c.noalias() = C * qd;

    dyn->calcGravityVector(jacobians, adjoints, g);

    dyn->calcFrictionVector(f);

    modelTorque.noalias() = M * qdd;
    modelTorque.noalias() += C * qd;
    modelTorque.noalias() += g + f;

    dyn->calcSlotineLiRegressor(jacobians, jacobianDots, adjs, adjoints, qd, qdd, regressor);

    c->getParams(params);

    Eigen::internal::set_is_malloc_allowed(true);
    t2 = steady_clock::now();

    duration<double> time_span = duration_cast<duration<double> >(t2 - t1);

    std::cout << "It took ARDL " << time_span.count() << " seconds.";
    std::cout << std::endl;
    LOG_DEBUG_LEVEL6("TURNING ON MALLOC 1");
    // LOG_DEBUG_LEVEL1("PARAMS DL: \n {} \n\n", regressor);
    LOG_DEBUG_LEVEL1("Friction : modelTorque DL: \n {} \n\n", modelTorque);
    // LOG_DEBUG_LEVEL1("Friction : regressorTorque DL: \n {} \n\n", regressor * params);

    LOG_DEBUG_LEVEL1("RBDL modelTorque: \n {} \n\n", modelTorqueRBDL);

    checkApproxMatrix(modelTorque, regressor * params, 1e-10);
    checkApproxMatrix(modelTorque - f, modelTorqueRBDL, 1e-10);

    // Eigen::VectorXd f(c->getNumOfJoints());
    // dyn->calcFrictionVector(f);
    // LOG_DEBUG_LEVEL1("f DL: \n {} \n\n", f);
}

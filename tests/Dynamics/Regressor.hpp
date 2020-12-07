#pragma once
#define EIGEN_RUNTIME_NO_MALLOC
#include "robotTestMain.cpp"
#include "Dynamics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"
#include <kdl/chainfksolvervel_recursive.hpp>
#include <valgrind/callgrind.h>

using namespace ARDL::Util;

size_t iters= 5000;
// size_t iters = 1;
TEST_CASE("Standard Regressor Check", "[adjoint][kinematics]") {
#if ARDL_VARIANT
    std::cout << "VARIANT ENABLED" << std::endl;
#endif
    using T= double;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    std::shared_ptr<Chain<T>> c;
    std::shared_ptr<ForwardKinematics<T>> fk;
    std::shared_ptr<Dynamics<T>> dyn;

#if !ARDL_EXTERNAL_DATA
    setupARDL(urdfModel, c, fk, dyn);
#else
    ARDL::Data<double> data(urdfModel);
    setupARDL(urdfModel, data, c, fk, dyn);
#endif

    size_t dof= c->getNumOfJoints();
    std::cout << dof << std::endl;

    std::cout << pModel.nq << std::endl;
    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<Pose<T>> adjoints;
    aligned_vector<Motion<T>> adjs;
    aligned_vector<Jacobian<T>> jacobians, jacobianDots;
    aligned_vector<aligned_vector<Jacobian<T>>> jacobiansDq, jacobianDotsDq;
    Regressor<T> Y;
    aligned_vector<Regressor<T>> regressorDq, regressorDqd, regressorDqdd;
    VectorX<T> params(dof * 10);

    c->getParams(params);
    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    qd.setZero();
    qdd.setZero();
    adjoints.resize(dof + 1);
    adjs.resize(dof);

    ARDL::Util::init(jacobians, c->getNumOfJoints());
    ARDL::Util::init(jacobianDots, c->getNumOfJoints());
    ARDL::Util::init(jacobiansDq, c->getNumOfJoints());
    ARDL::Util::init(jacobianDotsDq, c->getNumOfJoints());
    ARDL::Util::init(Y, c->getNumOfJoints());
    ARDL::Util::init(regressorDq, c->getNumOfJoints());
    ARDL::Util::init(regressorDqd, c->getNumOfJoints());
    ARDL::Util::init(regressorDqdd, c->getNumOfJoints());

    REFRESH("ARDL Y");
    for(size_t i= 0; i < iters; i++) {
        randomJointState(q, qd, qdd);
        c->updateChain(q, qd);
        c->updateMatricesOptim();
        fk->getBodyAdjointsOptim(adjoints);
        fk->getJacobians<Frame::BODY>(adjoints, jacobians);
        fk->getLieBrackets(adjs, jacobians);
        fk->getJacobianDots<Frame::BODY>(adjoints, adjs, jacobians, jacobianDots);
        dyn->calcSlotineLiRegressor(qd, qdd, adjoints, adjs, jacobians, jacobianDots, Y);
    }
    TOC("ARDL Y");
    REFRESH("PIN Y");
    for(size_t i= 0; i < iters; i++) {
        randomJointState(q, qd, qdd);
        Y= pinocchio::computeJointTorqueRegressor(pModel, pData, q, qd, qdd);
    }
    TOC("PIN Y");

    // CALLGRIND_START_INSTRUMENTATION;
    // pinocchio::computeJointTorqueRegressor(pModel, pData, q, qd, qdd);
    // CALLGRIND_STOP_INSTRUMENTATION;
    // due to different order of parameters
    for(size_t i= 1; i < pModel.njoints; ++i)
        params.segment<10>((int) ((i - 1) * 10))= pModel.inertias[i].toDynamicParameters();

    Eigen::VectorXd tauPinReg= (Y * params).transpose();
    Eigen::VectorXd tauPin= pinocchio::rnea(pModel, pData, q, qd, qdd).transpose();
    CALLGRIND_START_INSTRUMENTATION;
    c->updateChain(q, qd);
    // c->updateMatrices();
    // fk->getBodyAdjoints(adjoints);
    c->updateMatricesOptim();
    fk->getBodyAdjointsOptim(adjoints);
    fk->getJacobians<Frame::BODY>(adjoints, jacobians);
    fk->getLieBrackets(adjs, jacobians);
    fk->getJacobianDots<Frame::BODY>(adjoints, adjs, jacobians, jacobianDots);
    dyn->calcSlotineLiRegressor(qd, qdd, adjoints, adjs, jacobians, jacobianDots, Y);
    CALLGRIND_STOP_INSTRUMENTATION;

    c->getParams(params);
    Eigen::VectorXd tauARDL= (Y * params).transpose();
    checkApproxMatrix(tauPinReg, tauPin, 1e-6);
    checkApproxMatrix(tauPinReg, tauARDL, 1e-6);
    CALLGRIND_DUMP_STATS;
    std::cout << "ARDL " << tauARDL.transpose() << std::endl;
    std::cout << "PIN " << tauPin.transpose() << std::endl;
}

TEST_CASE("Base Regressor Check", "[adjoint][kinematics]") {
#if ARDL_VARIANT
    std::cout << "VARIANT ENABLED" << std::endl;
#endif
    using T= float;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    std::shared_ptr<Chain<T>> c;
    std::shared_ptr<ForwardKinematics<T>> fk;
    std::shared_ptr<Dynamics<T>> dyn;

#if !ARDL_EXTERNAL_DATA
    setupARDL(urdfModel, c, fk, dyn);
#else
    ARDL::Data<double> data(urdfModel);
    setupARDL(urdfModel, data, c, fk, dyn);
#endif

    size_t dof= c->getNumOfJoints();
    std::cout << dof << std::endl;

    std::cout << pModel.nq << std::endl;
    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<Pose<T>> adjoints;
    aligned_vector<Motion<T>> adjs;
    aligned_vector<Jacobian<T>> jacobians, jacobianDots;
    aligned_vector<aligned_vector<Jacobian<T>>> jacobiansDq, jacobianDotsDq;
    Regressor<T> Y;
    BaseRegressor<T> Yb;
    VectorX<T> params(dof * 10);

    c->getParams(params);
    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    qd.setZero();
    qdd.setZero();
    adjoints.resize(dof + 1);
    adjs.resize(dof);

    ARDL::Util::init(jacobians, c->getNumOfJoints());
    ARDL::Util::init(jacobianDots, c->getNumOfJoints());
    ARDL::Util::init(jacobiansDq, c->getNumOfJoints());
    ARDL::Util::init(jacobianDotsDq, c->getNumOfJoints());
    ARDL::Util::init(Y, c->getNumOfJoints());

    dyn->calcBaseProjection(100000, M_PI, 1e-10);
    Yb.resize(c->getNumOfJoints(), dyn->getNumOfBaseParams());

    VectorX<T> baseParams= dyn->getParameterProjector() * params;

    Eigen::VectorXd tauPinReg(dof);
    ARDL::VectorX<T> tauARDL(dof);

    for(size_t i= 1; i < pModel.njoints; ++i)
        params.segment<10>((int) ((i - 1) * 10))= pModel.inertias[i].toDynamicParameters().cast<T>();

    for(size_t i= 0; i < iters; i++) {
        randomJointState(q, qd, qdd);
        c->updateChain(q, qd);
        c->updateMatricesOptim();
        fk->getBodyAdjointsOptim(adjoints);
        fk->getJacobians<Frame::BODY>(adjoints, jacobians);
        fk->getLieBrackets(adjs, jacobians);
        fk->getJacobianDots<Frame::BODY>(adjoints, adjs, jacobians, jacobianDots);
        dyn->calcSlotineLiRegressor(qd, qdd, adjoints, adjs, jacobians, jacobianDots, Y);
        Yb= Y * dyn->getRegressorProjector();

        Y= pinocchio::computeJointTorqueRegressor(pModel, pData, q.cast<double>(), qd.cast<double>(), qdd.cast<double>()).cast<T>();
        tauPinReg= (Y * params).cast<double>();
        tauARDL= (Yb * baseParams);
    checkApproxMatrix(tauPinReg.cast<T>(), tauARDL, 1e-4);
    }
    std::cout << "ARDL " << tauARDL.transpose() << std::endl;
    std::cout << "PIN " << tauPinReg.transpose() << std::endl;
    std::cout<<Yb<<std::endl << std::endl;

    std::cout << c->getNumOfJoints()*10 << std::endl;
    std::cout << dyn->getNumOfBaseParams() << std::endl;
}

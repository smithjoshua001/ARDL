#pragma once
// #define EIGEN_USE_BLAS ON
#include <ostream>
#define EIGEN_RUNTIME_NO_MALLOC ON
// #define ARDL_NO_VARIANT ON
#include "robotTestMain.cpp"
#include "Kinematics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"
#include "pinocchio/algorithm/frames.hpp"

using namespace ARDL::Util;

TEST_CASE("Adjoint Chain Test", "[adjoint][kinematics]") {
    std::shared_ptr<Chain<double>> c;
    std::shared_ptr<ForwardKinematics<double>> fk;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    setupARDL(urdfModel, c, fk);

    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints()), changeTmp(c->getNumOfJoints());

    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    aligned_vector<AdjointSE3<double>> adjoints;
    adjoints.resize(c->getNumOfJoints() + 1);

    size_t dof= c->getNumOfJoints();
    pinocchio::computeAllTerms(pModel, pData, q, qd);
    c->updateChain(q, qd);
    c->updateMatricesOptim();
    fk->getAdjointsOptim(adjoints);
    for(size_t i= 0; i < pData.oMi.size(); i++) {
        std::cout << adjoints[i] << std::endl;
        std::cout << pData.oMi[i] << std::endl;
    }
    pinocchio::framesForwardKinematics(pModel, pData, q);
    std::cout<<pData.oMf.back()<<std::endl;
    AdjointSE3<double> ee;
    fk->getEEAdjoint(adjoints,ee);
    std::cout<<ee<<std::endl;

    for(size_t i= 0; i < pData.oMi.size(); i++) {
        checkApproxMatrix(adjoints[i].getR(), pData.oMi[i].rotation(), 1e-6);
        checkApproxMatrix(adjoints[i].getP(), pData.oMi[i].translation(), 1e-6);
    }
}
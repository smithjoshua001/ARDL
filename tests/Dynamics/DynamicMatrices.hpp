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

// size_t iters = 1;
TEST_CASE("Matrix Check", "[adjoint][kinematics]") {

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

    std::cout<<"NUM OF MOVABLE JOINTS: "<<c->getMoveableLinksRef().size()<<std::endl;
    size_t dof= c->getNumOfJoints();
    std::cout << dof << std::endl;

    std::cout << pModel.nq << std::endl;
    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<AdjointSE3<T>> adjoints;
    aligned_vector<LieBracketSE3<T>> adjs;
    aligned_vector<Jacobian<T>> jacobians, jacobianDots;
    aligned_vector<aligned_vector<Jacobian<T>>> jacobiansDq, jacobianDotsDq;

    Eigen::VectorXd G(dof), GP(dof), CP(dof), MP(dof);
    Eigen::MatrixXd M(dof, dof), C(dof, dof), CMP(dof,dof);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    adjoints.resize(dof + 1);
    adjs.resize(dof);

    ARDL::Util::init(jacobians, c->getNumOfJoints());
    ARDL::Util::init(jacobianDots, c->getNumOfJoints());
    ARDL::Util::init(jacobiansDq, c->getNumOfJoints());
    ARDL::Util::init(jacobianDotsDq, c->getNumOfJoints());
    ARDL::Util::init(M, c->getNumOfJoints());
    ARDL::Util::init(C, c->getNumOfJoints());
    ARDL::Util::init(G, c->getNumOfJoints());
    ARDL::Util::init(MP, c->getNumOfJoints());
    ARDL::Util::init(CP, c->getNumOfJoints());
    ARDL::Util::init(GP, c->getNumOfJoints());

    GP= pinocchio::computeGeneralizedGravity(pModel, pData, q);

    c->updateChain(q, qd);
    c->updateMatricesOptim();
    fk->getBodyAdjointsOptim(adjoints);
    fk->getJacobians<Frame::BODY>(adjoints, jacobians);
    dyn->calcGravityVectorOptim(jacobians, adjoints, G);
    std::cout << "PIN: " << GP.transpose() << std::endl;
    std::cout << "ARDL: " << G.transpose() << std::endl;

    checkApproxMatrix(GP, G, 1e-6);

    CMP = pinocchio::computeCoriolisMatrix(pModel,pData,q,qd);
    CP = CMP*qd;

    fk->getLieBrackets(adjs, jacobians);
    fk->getJacobianDots<Frame::BODY>(adjoints, adjs, jacobians, jacobianDots);
    dyn->calcCoriolisMatrixOptim(jacobians, jacobianDots, adjs,adjoints, C);

    std::cout << "PIN: " << CP.transpose() << std::endl;
    std::cout<<"ARDL: "<<(C*qd).transpose()<<std::endl;


    checkApproxMatrix(CP, C*qd, 1e-6);

    MP = pinocchio::rnea(pModel,pData,q,qd,qdd) - CP - GP;

    dyn->calcJointInertiaMatrixOptim(adjoints,jacobians,M);


    std::cout << "PIN: " << MP.transpose() << std::endl;
    std::cout<<"ARDL: "<<(M*qdd).transpose()<<std::endl;


    checkApproxMatrix(MP, (M*qdd), 1e-6);

}

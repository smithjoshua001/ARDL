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

TEST_CASE("Matrix Check Tree", "[adjoint][dynamics]") {

    using T= double;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    std::shared_ptr<Tree<T>> t;
    std::shared_ptr<ForwardKinematicsTree<T>> fk;
    std::shared_ptr<DynamicsTree<T>> dyn;

    setupARDL(urdfModel,t,fk,dyn);

    std::cout<<"NUM OF MOVABLE JOINTS: "<<t->getMoveableLinksRef().size()<<std::endl;
    size_t dof= t->getNumOfJoints();
    std::cout << dof << std::endl;

    std::cout << pModel.nq << std::endl;
    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<Pose<T>> adjoints;
    aligned_vector<Motion<T>> adjs;
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

    ARDL::Util::init(jacobians, t->getNumOfJoints());
    ARDL::Util::init(jacobianDots, t->getNumOfJoints());
    ARDL::Util::init(jacobiansDq, t->getNumOfJoints());
    ARDL::Util::init(jacobianDotsDq, t->getNumOfJoints());
    ARDL::Util::init(M, t->getNumOfJoints());
    ARDL::Util::init(C, t->getNumOfJoints());
    ARDL::Util::init(G, t->getNumOfJoints());
    ARDL::Util::init(MP, t->getNumOfJoints());
    ARDL::Util::init(CP, t->getNumOfJoints());
    ARDL::Util::init(GP, t->getNumOfJoints());

    GP= pinocchio::computeGeneralizedGravity(pModel, pData, q);

    t->update(q, qd);
    t->updateMatricesOptim();
    fk->getPosesOptim<Frame::SPATIAL>(adjoints);
    fk->getJacobians<Frame::SPATIAL>(adjoints, jacobians);
    dyn->calcGravityVectorOptim<Frame::SPATIAL>(jacobians, adjoints, G);
    std::cout << "PIN: " << GP.transpose() << std::endl;
    std::cout << "ARDL: " << G.transpose() << std::endl;

    // checkApproxMatrix(GP, G, 1e-6);

    CMP = pinocchio::computeCoriolisMatrix(pModel,pData,q,qd);
    CP = CMP*qd;

    fk->getVelocities(adjs, jacobians);
    fk->getJacobianDots<Frame::SPATIAL>(adjoints, adjs, jacobians, jacobianDots);
    dyn->calcCoriolisMatrixOptim(jacobians, jacobianDots, adjs,adjoints, C);

    std::cout << "PIN: " << CP.transpose() << std::endl;
    std::cout<<"ARDL: "<<(C*qd).transpose()<<std::endl;


    // checkApproxMatrix(CP, C*qd, 1e-6);

    MP = pinocchio::rnea(pModel,pData,q,qd,qdd) - CP - GP;

    dyn->calcJointInertiaMatrixOptim<Frame::SPATIAL>(adjoints,jacobians,M);


    std::cout << "PIN: " << MP.transpose() << std::endl;
    std::cout<<"ARDL: "<<(M*qdd).transpose()<<std::endl;
    // Eigen::VectorXd tmp(10);
    // for(size_t i = 0;i<t->getNumOfJoints();i++){
    //     // std::cout<<pData.oYcrb[i]<<std::endl<<std::endl;

    //     std::cout<<"PIN "<<pModel.inertias[i+1].matrix()<<std::endl<<std::endl;
    //     t->getMoveableLinksRef()[i+1]->getSI().toVector(tmp);
    //     std::cout<<"ARDL "<<tmp.transpose()<<std::endl<<std::endl;
    // }

    // checkApproxMatrix(MP, (M*qdd), 1e-6);

    MatrixX<double> regressor(dof,dof*10);
    VectorX<double> params(dof*10);
    MatrixX<double> compare(dof*10,2);
    for(size_t i= 1; i < pModel.njoints; ++i)
        params.segment<10>((int) ((i - 1) * 10))= pModel.inertias[i].toDynamicParameters().cast<T>();

    compare.col(0) = params;
    t->getParams(params);
    compare.col(1) = params;
    std::cout<<compare<<std::endl<<std::endl;
    dyn->calcSlotineLiRegressor(qd,qdd, adjoints,adjs,jacobians,jacobianDots,regressor);
    std::cout<<(M*qdd+C*qd+G).transpose()<<std::endl; 
    std::cout<<(regressor*params).transpose()<<std::endl;
    dyn->calcBaseProjection(10000, M_PI, 1e-5);
    MatrixX<double> Yb;
    Yb.resize(t->getNumOfJoints(), dyn->getNumOfBaseParams());

    VectorX<double> baseParams= dyn->getParameterProjector() * params;
 Yb= regressor * dyn->getRegressorProjector();


    std::cout<<(Yb*baseParams).transpose()<<std::endl;


    std::cout<<dyn->getNumOfBaseParams()<<std::endl;
}


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

    aligned_vector<Pose<T>> adjoints;
    aligned_vector<Motion<T>> adjs;
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

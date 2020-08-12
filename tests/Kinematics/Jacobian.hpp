#pragma once
// #define EIGEN_USE_BLAS ON
#define EIGEN_RUNTIME_NO_MALLOC ON
// #define ARDL_NO_VARIANT ON
#include "robotTestMain.cpp"
#include "Kinematics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"
#include "pinocchio/algorithm/frames.hpp"

using namespace ARDL::Util;

TEST_CASE("Jacobian Test", "[adjoint][kinematics]") {
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
    aligned_vector<Jacobian<double>> jacobians, jacobianDots, jacobiansP, jacobianDotsP;
    jacobians.resize(c->getNumOfJoints());
    jacobianDots.resize(c->getNumOfJoints());
    jacobiansP.resize(dof);
    jacobianDotsP.resize(dof);
    for(int i= 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).resize(c->getNumOfJoints());
        jacobians.at(i).setZero();
        jacobiansP[i].resize(dof);
        jacobiansP[i].setZero();
        jacobianDots.at(i).resize(c->getNumOfJoints());
        jacobianDots.at(i).setZero();
        jacobianDotsP[i].resize(dof);
        jacobianDotsP[i].setZero();
    }

    aligned_vector<LieBracketSE3<double>> adjs;
    adjs.resize(c->getNumOfJoints());

    ARDL::Dynamics<double> dyn(c);
    constexpr ARDL::Frame frame= ARDL::Frame::SPATIAL;
    c->updateChain(q, qd);
    c->updateMatrices();
    if constexpr(frame == ARDL::Frame::SPATIAL) {
        fk->getAdjoints(adjoints);
    } else if constexpr(frame == ARDL::Frame::BODY) {
        fk->getBodyAdjoints(adjoints);
    }
    for(size_t i= 0; i < adjoints.size(); i++) {
        std::cout << "Adjoint " << i << std::endl;
        std::cout << adjoints[i] << std::endl;
    }
    c->updateMatricesOptim();
    if constexpr(frame == ARDL::Frame::SPATIAL) {
        fk->getAdjointsOptim(adjoints);
    } else if constexpr(frame == ARDL::Frame::BODY) {
        fk->getBodyAdjointsOptim(adjoints);
    }
    for(size_t i= 0; i < adjoints.size(); i++) {
        std::cout << "AdjointOptim " << i << std::endl;
        std::cout << adjoints[i] << std::endl;
    }
    fk->getJacobians<frame>(adjoints, jacobians);
    pinocchio::forwardKinematics(pModel, pData, q, qd, qdd);

    pinocchio::computeAllTerms(pModel, pData, q, qd);
    pinocchio::computeForwardKinematicsDerivatives(pModel, pData, q, qd, qdd);
    pinocchio::computeJointJacobiansTimeVariation(pModel, pData, q, qd);
    pinocchio::computeJointJacobians(pModel, pData);
    pinocchio::computeJointJacobians(pModel, pData);
    for(size_t i= 0; i < dof; i++) {
        pinocchio::getJointJacobian(pModel, pData, i + 1, pinocchio::WORLD, jacobiansP[i]);
    }
    std::cout<<"ARDL \n"<<jacobians[0]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[0]<<std::endl<<std::endl;

    std::cout<<"ARDL \n"<<jacobians[1]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[1]<<std::endl<<std::endl;

    std::cout<<"ARDL \n"<<jacobians[2]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[2]<<std::endl<<std::endl;
    std::cout<<"ARDL \n"<<jacobians[3]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[3]<<std::endl<<std::endl;
    std::cout<<"ARDL \n"<<jacobians[4]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[4]<<std::endl<<std::endl;
    std::cout<<"ARDL \n"<<jacobians[5]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[5]<<std::endl<<std::endl;
    std::cout<<"ARDL \n"<<jacobians[6]<<std::endl<<std::endl;
    std::cout<<"PIN \n"<<jacobiansP[6]<<std::endl<<std::endl;
    for(size_t i= 0; i < dof; i++) { checkApproxMatrix(jacobians[i], jacobiansP[i], 1e-6); }
pinocchio::getFrameJacobian(pModel, pData, pModel.frames.size()-1, pinocchio::WORLD, jacobiansP[6]);
    std::cout<<"PIN \n"<<jacobiansP[6]<<std::endl<<std::endl;
}

TEST_CASE("JD1", "[adjoint][kinematics]") {
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
    aligned_vector<Jacobian<double>> jacobians, jacobianDots, jacobiansP, jacobianDotsP;
    jacobians.resize(c->getNumOfJoints());
    jacobianDots.resize(c->getNumOfJoints());
    jacobiansP.resize(dof);
    jacobianDotsP.resize(dof);
    for(int i= 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).resize(c->getNumOfJoints());
        jacobians.at(i).setZero();
        jacobiansP[i].resize(dof);
        jacobiansP[i].setZero();
        jacobianDots.at(i).resize(c->getNumOfJoints());
        jacobianDots.at(i).setZero();
        jacobianDotsP[i].resize(dof);
        jacobianDotsP[i].setZero();
    }

    aligned_vector<LieBracketSE3<double>> adjs;
    adjs.resize(c->getNumOfJoints());

    ARDL::Dynamics<double> dyn(c);
    constexpr ARDL::Frame frame= ARDL::Frame::BODY;
    c->updateChain(q, qd);
    c->updateMatrices();
    fk->getBodyAdjoints(adjoints);
    fk->getJacobians<frame>(adjoints, jacobians);

    fk->getLieBrackets(adjs, jacobians);

    fk->getJacobianDots<frame>(adjoints, adjs, jacobians, jacobianDots);
    for(size_t i= 0; i < dof; i++) {
        std::cout << "JD " << i << std::endl;
        std::cout << jacobianDots[i] << std::endl << std::endl;
        // std::cout<<jacobianDotsP[i]<<std::endl<<std::endl;
    }

    for(int i= 0; i < c->getNumOfJoints(); i++) {
        jacobians.at(i).setZero();
        jacobiansP[i].setZero();
        jacobianDots.at(i).setZero();
        jacobianDotsP[i].setZero();
    }
    c->updateMatricesOptim();
    if constexpr(frame == ARDL::Frame::SPATIAL) {
        // fk->getAdjointsOptim(adjoints);
    } else if constexpr(frame == ARDL::Frame::BODY) {
        fk->getBodyAdjointsOptim(adjoints);
    }
    fk->getJacobians<frame>(adjoints, jacobians);

    fk->getLieBrackets(adjs, jacobians);

    fk->getJacobianDots<frame>(adjoints, adjs, jacobians, jacobianDots);
    pinocchio::forwardKinematics(pModel, pData, q, qd, qdd);

    pinocchio::computeAllTerms(pModel, pData, q, qd);
    pinocchio::computeForwardKinematicsDerivatives(pModel, pData, q, qd, qdd);
    pinocchio::computeJointJacobians(pModel, pData);
    pinocchio::computeJointJacobiansTimeVariation(pModel, pData, q, qd);
    for(size_t i= 0; i < dof; i++) {
        pinocchio::getJointJacobian(pModel, pData, i + 1, pinocchio::LOCAL, jacobiansP[i]);

        pinocchio::getJointJacobianTimeVariation(pModel, pData, i + 1, pinocchio::LOCAL, jacobianDotsP[i]);
    }
    for(size_t i= 0; i < dof; i++) {
        checkApproxMatrix(jacobians[i], jacobiansP[i], 1e-6);
        std::cout << "JD " << i << std::endl;
        std::cout << jacobianDots[i] << std::endl << std::endl;
        std::cout << jacobianDotsP[i] << std::endl << std::endl;
    }
}
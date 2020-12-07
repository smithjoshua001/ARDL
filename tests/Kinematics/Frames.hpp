#pragma once
// #define EIGEN_USE_BLAS ON
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <ostream>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fwd.hpp>
#define EIGEN_RUNTIME_NO_MALLOC ON
// #define ARDL_NO_VARIANT ON
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"
#include "Kinematics/common.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "robotTestMain.cpp"

#include "manif/SE3.h"

#include "ARDL/Util/MatrixInitializer.hpp"
using namespace ARDL;
using namespace ARDL::Util;

TEST_CASE("Tree Pose Test", "[pose][kinematics]") {
    std::shared_ptr<Tree<double>> t;
    std::shared_ptr<ForwardKinematicsTree<double>> fk;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    setupARDL(urdfModel, t, fk);
    size_t dof = t->getNumOfJoints();
    Eigen::VectorXd q(dof);
    Eigen::VectorXd qd(dof), qdd(dof), changeTmp(dof);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    t->random();
    q = t->getQ();
    qd = t->getQd();
    t->random();
    randomJointState(q, qd, qdd);
    t->update(q, qd);
    t->updateMatricesOptim();
    REQUIRE(dof == pModel.nq);
    aligned_vector<Pose<double>> poses(dof + 1);
    fk->getPosesOptim<ARDL::Frames::SPATIAL>(poses);
    std::cout << "Q " << q.transpose() << std::endl;
    std::cout << "QD " << qd.transpose() << std::endl;

    pinocchio::computeAllTerms(pModel, pData, q, qd);
    pinocchio::framesForwardKinematics(pModel, pData, q);
    for (size_t i = 0; i < pData.oMi.size(); i++) {
        // checkApproxMatrix(adjoints[i].getR(), pData.oMi[i].rotation(), 1e-6);
        // checkApproxMatrix(adjoints[i].getP(), pData.oMi[i].translation(), 1e-6);
        std::cout << "=============================" << i
                  << "=============================" << std::endl;
        std::cout << "PIN " << pData.oMi[i] << std::endl;
        std::cout << "ARDL " << poses[i] << std::endl << std::endl;

        std::cout << "PIN " << pData.liMi[i] << std::endl;
        if (i != 0)
            std::cout << "ARDL "
                      << ARDL_visit(
                             t->getMoveableLinksRef()[i]->getParentJoint(),
                             getAdjointLocal())
                      << std::endl
                      << std::endl;
    }

    aligned_vector<Motion<double>> vels;
    aligned_vector<Jacobian<double>> jacobians, jacobianDots, jacobiansP,
        jacobianDotsP;
    aligned_vector<aligned_vector<Jacobian<double>>> jdq;
    init(jacobians, dof);
    init(jacobianDots, dof);
    init(jacobiansP, dof);
    init(jacobianDotsP, dof);
    init(jdq, dof);

    vels.resize(dof);

    fk->getJacobians<ARDL::Frames::SPATIAL>(poses, jacobians);
    fk->getVelocities(vels, jacobians);
    fk->getJacobianDots<ARDL::Frames::SPATIAL>(poses, vels, jacobians,
                                               jacobianDots);

    pinocchio::ReferenceFrame pinFrame;
    pinFrame = pinocchio::ReferenceFrame::WORLD;

    pinocchio::computeJointJacobiansTimeVariation(pModel, pData, q, qd);

    for (size_t i = 0; i < dof; i++) {
        pinocchio::getJointJacobian(pModel, pData, i + 1, pinFrame,
                                    jacobiansP[i]);
        pinocchio::getJointJacobianTimeVariation(pModel, pData, i + 1, pinFrame,
                                                 jacobianDotsP[i]);

        std::cout << "PIN " << jacobiansP[i] << std::endl;
        std::cout << "ARDL " << jacobians[i] << std::endl << std::endl;
        std::cout << "PIN D " << jacobianDotsP[i] << std::endl;
        std::cout << "ARDL D " << jacobianDots[i] << std::endl << std::endl;
    }

   pinocchio::computeJointJacobians(pModel,pData,q);
    pinocchio::computeJointKinematicHessians(pModel,pData,q);
    fk->getJacobiansDq(poses,jacobians,jdq);
    q[1] += 1e-3 ;
    aligned_vector<Jacobian<double>> jacobians2, jacobiansP2;

    init(jacobians2, dof);
    init(jacobianDots, dof);
    init(jacobiansP2, dof);
    init(jacobianDotsP, dof);

    for (size_t i = 0; i < dof; i++) {
        vels[i].getVelocity().setZero();
    }

    aligned_vector<Pose<double>> poses2(dof + 1);

    t->update(q, qd);
    t->updateMatricesOptim();

    fk->getPosesOptim<ARDL::Frames::SPATIAL>(poses2);

    fk->getJacobians<ARDL::Frames::SPATIAL>(poses2, jacobians2);
    fk->getVelocities(vels, jacobians2);
    fk->getJacobianDots<ARDL::Frames::SPATIAL>(poses2, vels, jacobians2,
                                               jacobianDots);
    Pose<double> pose3;
     poses[5].applyInverseTo(poses2[5],pose3);
    std::cout << poses2[5].getMatrixRef() - poses[5].getMatrixRef() << std::endl
              << std::endl;
              std::cout<<poses[5]<<std::endl;
              std::cout<<poses2[5]<<std::endl;
              std::cout<<pose3<<std::endl<<std::endl;
    Motion<double> vel;
    vel.getVelocity() = jacobians[4].col(0);
    std::cout<<vel.getMatrix()<<std::endl<<std::endl;
    std::cout<<poses[4].getMatrixRef()<<std::endl<<std::endl;
    Eigen::Matrix<double, 6, 6> pdq = poses[4].getMatrixRef() * vel.getMatrix();
    std::cout << pdq * 0.1 << std::endl<<std::endl;
    Eigen::Quaterniond quat(poses[4].getR()), quat2(poses2[4].getR());
    manif::SE3d X(poses[4].getP(),quat), X2(poses2[4].getP(),quat2);
    quat = Eigen::Quaterniond(poses[1].getR());
    manif::SE3d X3(poses[1].getP(), quat);
    manif::SE3Tangentd v;
    v.coeffs() = jacobians[3].col(1);
    std::cout<<v.coeffs()<<std::endl;

    // std::cout<<X.compose((X.plus(v*0.1))).adj()<<std::endl;

    std::cout<<((X.plus(v))).adj()*1e-3<<std::endl<<std::endl;
    std::cout<<X.compose(v.exp()).adj()<<std::endl<<std::endl;
    std::cout<<X.adj()*v.smallAdj()*1e-3<<std::endl<<std::endl;
    std::cout<<v.smallAdj()*X.adj()*1e-3<<std::endl<<std::endl;
    std::cout<<(X.inverse()*X2).adj()<<std::endl<<std::endl;
    std::cout<<(X2.adj()-X.adj())<<std::endl<<std::endl;


    std::cout<<X<<std::endl<<std::endl;
    std::cout<<X2<<std::endl<<std::endl;
    std::cout<<X.inverse()*X2<<std::endl<<std::endl;

    for (size_t i = 0; i < dof; i++) {

    // std::cout<<"DQ "<<jdq[i][1]*0.05<<std::endl;
    //     // pinocchio::getJointJacobian(pModel, pData, i + 1, pinFrame,
    //     //                             jacobiansP[i]);
    //     // pinocchio::getJointJacobianTimeVariation(pModel, pData, i + 1, pinFrame,
    //     //                     jacobianDotsP[i]);

    auto tensor = pinocchio::getJointKinematicHessian(pModel,pData,i+1,pinocchio::WORLD);

    pinocchio::computeJointJacobiansTimeVariation(pModel, pData, q, qd);

    pinocchio::getJointJacobian(pModel, pData, i + 1, pinFrame,
                                    jacobiansP2[i]);
    // std::cout<<tensor<<std::endl;
Eigen::Map<pinocchio::Data::Matrix6x> dJ_dq(tensor.data() + 1 * pModel.nv*6,6,pModel.nv);
        std::cout << "PIN " << dJ_dq*1e-3 << std::endl;
        std::cout << "ARDL " << jacobians2[i]-jacobians[i] << std::endl << std::endl;
        std::cout<<"PIN E "<< jacobiansP2[i]-jacobiansP[i]<<std::endl<<std::endl;
        std::cout << "ARDL " << jdq[i][1]*1e-3<< std::endl << std::endl;

    //     std::cout << "ARDL " << jacobians2[i] << std::endl << std::endl;
    //     // std::cout << "PIN D " << jacobianDotsP[i] << std::endl;
    //     std::cout << "ARDL D " << jacobianDots[i] << std::endl << std::endl;
    }

    // pinocchio::computeFrameJacobian(pModel, pData, q, 7, jacobiansP[6]);
    // std::cout << "PIN SAN \n" << jacobiansP[6] << std::endl << std::endl;

}

TEST_CASE("Adjoint Chain Test", "[adjoint][kinematics]") {
    std::shared_ptr<Chain<double>> c;
    std::shared_ptr<ForwardKinematics<double>> fk;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    setupARDL(urdfModel, c, fk);

    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints()),
        changeTmp(c->getNumOfJoints());

    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    aligned_vector<Pose<double>> adjoints;
    adjoints.resize(c->getNumOfJoints() + 1);

    size_t dof = c->getNumOfJoints();
    pinocchio::computeAllTerms(pModel, pData, q, qd);
    c->updateChain(q, qd);
    c->updateMatricesOptim();
    fk->getAdjointsOptim(adjoints);
    for (size_t i = 0; i < pData.oMi.size(); i++) {
        std::cout << adjoints[i] << std::endl;
        std::cout << pData.oMi[i] << std::endl;
    }
    pinocchio::framesForwardKinematics(pModel, pData, q);
    std::cout << pData.oMf.back() << std::endl;
    Pose<double> ee, worldAd;
    fk->getEEAdjoint(adjoints, ee);
    std::cout << ee << std::endl;

    for (size_t i = 0; i < pData.oMi.size(); i++) {
        checkApproxMatrix(adjoints[i].getR(), pData.oMi[i].rotation(), 1e-6);
        checkApproxMatrix(adjoints[i].getP(), pData.oMi[i].translation(), 1e-6);
    }

    adjoints[0].setIdentity();
    adjoints.back().setIdentity();
    fk->getBodyAdjointsOptim(adjoints);
    adjoints[0].applyInverse(adjoints.back(), worldAd);
    std::cout << adjoints.back() << std::endl;
    std::cout << worldAd << std::endl;

    adjoints[0].apply(worldAd, adjoints.back());
    std::cout << adjoints.back() << std::endl;
    fk->getEEAdjoint(worldAd, ee);
    std::cout << ee << std::endl;

    Quaterniond targ;
    targ.coeffs() << -0.161, -0.201, 0.936, -0.241;
    // Quaterniond current;
    // current.coeffs() << 0, 0.707, 0.707, 0;
    Pose<double> targetAd(targ.matrix(), Eigen::Vector3d(0.5, 0.5, 0.2));
    // Pose<double> currentAd(current.matrix(), Eigen::Vector3d(0, 0, 0));
    Pose<double> currentAd = worldAd;
    Pose<double> errorAd;
    currentAd.applyInverseTo(targetAd, errorAd);

    pinocchio::SE3 pt(targ, Eigen::Vector3d(0.5, 0.5, 0.2));
    pinocchio::SE3 pc(currentAd.getR().matrix(), currentAd.getP());
    pinocchio::SE3 perr = pc.actInv(pt);
    std::cout << "SANITY CHECK" << std::endl;
    std::cout << worldAd << std::endl;
    std::cout << errorAd << std::endl;
    std::cout << perr << std::endl;

    std::cout << std::endl
              << pinocchio::log6(perr).toVector().transpose() << std::endl
              << std::endl;

    Quaterniond errorQuat(errorAd.getR());
    errorQuat.normalize();
    // error.tail<3>() =20*current.w()*targ.vec() - targ.w() *current.vec() -
    // targ.vec().cross(current.vec());

    // Quaternion tmp = *(errorQuat*targ);
    // errorQuat.normalize();
    Eigen::Vector3d test;
    test = 2 * std::atan2(errorQuat.coeffs().head<3>().norm(), errorQuat.w()) *
           (errorQuat.coeffs().head<3>() / errorQuat.coeffs().head<3>().norm());

    std::cout << (errorQuat.inverse() * errorAd.getP()).transpose() << " "
              << test.transpose() << std::endl;

    //   Eigen::Matrix<double, 3, 4> conjugateQuat;
    //   conjugateQuat.col(0) = -errorQuat.coeffs().head<3>();
    //   ARDL::Util::Math::skewMatrix(errorQuat.coeffs().head<3>(),
    //                                conjugateQuat.block<3, 3>(0, 1));
    //   conjugateQuat.block<3, 3>(0, 1).diagonal().setConstant(errorQuat.w());
    //   std::cout << 2 * (conjugateQuat * errorQuat.coeffs()).array().transpose() << std::endl;

    std::cout << errorQuat.coeffs().head<3>().norm() << std::endl;
    std::cout << (2 *
                  (std::acos(errorQuat.w()) /
                   errorQuat.coeffs().head<3>().norm()) *
                  errorQuat.coeffs().head<3>())
                     .transpose()
              << std::endl;
    std::cout << std::acos((errorAd.getR().trace() - 1) / 2) << std::endl;
    double theta = std::acos((errorAd.getR().trace() - 1) / 2);
    std::cout << (theta / (2 * std::sin(theta))) *
                     (errorAd.getR() - errorAd.getR().transpose())
              << std::endl;
    Eigen::Matrix3d t4 = (theta / (2 * std::sin(theta))) *
                         (errorAd.getR() - errorAd.getR().transpose());
    Eigen::Vector3d t1 =
        (2 * (std::acos(errorQuat.w()) / errorQuat.coeffs().head<3>().norm()) *
         errorQuat.coeffs().head<3>());
    double t2 = errorQuat.coeffs().head<3>().norm();
    Eigen::Matrix3d t3 =
        Eigen::Matrix3d::Identity() - 0.5 * t4 +
        (1 - (theta * std::cos(theta / 2)) / (2 * std::sin(theta / 2))) /
            (theta * theta) * (t4 * t4);
    Eigen::Vector3d t5 = t3 * errorAd.getP();
    std::cout << std::endl << t5.transpose() << std::endl;

    std::cout << theta << " "
              << std::acos(
                     ((3 - 4 * (errorQuat.vec().array().pow(2).sum())) - 1) / 2)
              << " " << std::acos((2 * errorQuat.w() * errorQuat.w() - 1))
              << " " << errorAd.getR().trace() << " "
              << 4 * errorQuat.w() * errorQuat.w() - 1 << std::endl;

    std::cout << (2 *
                  (std::acos(-errorQuat.w()) /
                   errorQuat.coeffs().head<3>().norm()) *
                  -errorQuat.coeffs().head<3>())
                     .transpose()
              << std::endl;
    std::cout << errorQuat.coeffs().head<3>().norm() << std::endl;
    std::cout << std::acos(-errorQuat.w()) << std::endl;
    Quaterniond current(currentAd.getR());
    std::cout << current.coeffs().transpose() * targ.coeffs() << std::endl;
}

TEST_CASE("Velocity Chain Test", "[adjoint][kinematics]") {
    std::shared_ptr<Chain<double>> c;
    std::shared_ptr<ForwardKinematics<double>> fk;

    pinocchio::Model pModel;
    pinocchio::Data pData;

    setupPinocchio(urdfModel, pModel, pData);

    setupARDL(urdfModel, c, fk);

    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd qd(c->getNumOfJoints()), qdd(c->getNumOfJoints()),
        changeTmp(c->getNumOfJoints());

    q.setZero();
    qd.setZero();
    qdd.setZero();
    randomJointState(q, qd, qdd);

    aligned_vector<Pose<double>> adjoints;
    adjoints.resize(c->getNumOfJoints() + 1);

    aligned_vector<Motion<double>> adjs;
    aligned_vector<Jacobian<double>> jacobians;
    Jacobian<double> eeJac;

    adjs.resize(c->getNumOfJoints());

    ARDL::Util::init(jacobians, c->getNumOfJoints());
    ARDL::Util::init(eeJac, c->getNumOfJoints());

    size_t dof = c->getNumOfJoints();
    pinocchio::computeAllTerms(pModel, pData, q, qd);
    c->updateChain(q, qd);
    c->updateMatricesOptim();
    fk->getBodyAdjointsOptim(adjoints);
    fk->getJacobians<ARDL::Frames::BODY>(adjoints, jacobians);
    fk->getLieBrackets(adjs, jacobians);
    pinocchio::forwardKinematics(pModel, pData, q, qd, qdd);
    computeForwardKinematicsDerivatives(pModel, pData, q, qd, qdd);
    fk->getEEJacobian(jacobians, eeJac);

    for (size_t i = 0; i < adjs.size(); i++) {
        std::cout << adjs[i].getVelocity().transpose() << std::endl;
        std::cout << pData.v[i + 1] << std::endl;
        std::cout << pData.ov[i + 1] << std::endl;
        std::cout << pData.oMi[i + 1].act(pData.v[i + 1]) << std::endl;
        // std::cout<<<<std::endl;
    }
    Pose<double> ee;
    adjoints[0].applyInverse(adjoints.back(), ee);
    Quaterniond tmp(ee.getR().transpose());
    ARDL::Math::Motion<double> tmpLB, tmpLB2;
    // tmpLB.getVelocity() = eeJac*c->getQd();
    tmpLB.getVelocity() = adjs.back().getVelocity();
    ee.apply(tmpLB.getVelocity(), tmpLB2.getVelocity());
    // std::cout<<tmp._transformVector(pData.v.back().linear()).transpose()<<std::endl;
    std::cout << (tmpLB2.getVelocity()).transpose() << std::endl;
    // pinocchio::framesForwardKinematics(pModel, pData, q);
    // std::cout<<pData.oMf.back()<<std::endl;
    // Pose<double> ee, worldAd;
    // fk->getEEAdjoint(adjoints,ee);
    // std::cout<<ee<<std::endl;

    // for(size_t i= 0; i < pData.oMi.size(); i++) {
    //     checkApproxMatrix(adjoints[i].getR(), pData.oMi[i].rotation(), 1e-6);
    //     checkApproxMatrix(adjoints[i].getP(), pData.oMi[i].translation(),
    //     1e-6);
    // }

    // adjoints[0].setIdentity();
    // adjoints.back().setIdentity();
    // fk->getBodyAdjointsOptim(adjoints);
    // adjoints[0].applyInverse(adjoints.back(),worldAd);
    // fk->getEEAdjoint(worldAd,ee);
    // std::cout<<ee<<std::endl;
}
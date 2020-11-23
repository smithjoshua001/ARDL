#pragma once
#define EIGEN_RUNTIME_NO_MALLOC
// #include "robotTestMain.cpp"

#include "ARDL/Util/ModelPath.hpp"
#include "Collision/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"
#include <kdl/chainfksolvervel_recursive.hpp>
#include <valgrind/callgrind.h>
#include "Collision/MFTVisualizer.hpp"

using namespace ARDL::Util;

size_t iters= 5000;
// size_t iters = 1;
// TEST_CASE("Optimal Collision", "[adjoint][kinematics]") {
//     using T= double;

//     pinocchio::Model pModel;
//     pinocchio::Data pData;

//     setupPinocchio(urdfModel, pModel, pData);

//     std::shared_ptr<Chain<T>> c;
//     std::shared_ptr<ForwardKinematics<T>> fk;
//     std::shared_ptr<Dynamics<T>> dyn;

// #if !ARDL_EXTERNAL_DATA
//     setupARDL(urdfModel, c, fk, dyn);
// #else
//     ARDL::Data<double> data(urdfModel);
//     setupARDL(urdfModel, data, c, fk, dyn);
// #endif

//     size_t dof= c->getNumOfJoints();
//     std::cout << dof << std::endl;

//     std::cout << pModel.nq << std::endl;
//     VectorX<T> q(dof), qd(dof), qdd(dof);

//     aligned_vector<Pose<T>> adjoints;
//     adjoints.resize(dof + 1);

//     q.setZero();
//     qd.setZero();
//     qdd.setZero();
//     c->updateChain(q, qd);
//     // c->updateMatricesOptim();
//     // fk->getBodyAdjointsOptim(adjoints,true);
//     c->updateMatrices();
//     fk->getAdjoints(adjoints,true);
//     std::cout<<fk->collision()<<std::endl;

// }

std::string urdfModel= "";

int main(int argc, char *argv[]) {
    urdfModel= ARDL::Util::getModelFromGazeboPath(argv[1]);
    std::cout<<urdfModel<<std::endl;
    using T= double;
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

    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<Pose<T>> adjoints;
    adjoints.resize(dof + 1);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    // q<<0,0.5,0.1,-0.3,0.1,0.3,1.2;
    // q<<0,1.4,0.1,-2,0.1,0.3,1.2;

    // q<<0,0.5,0.1,-0.3,0.1,2,1.2;
    RobotVis rvis(urdfModel);
    rvis.setQ(q);
    rvis.run();
}

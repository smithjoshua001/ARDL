#pragma once

#include "robotTestMain.cpp"
#include "Model/common.hpp"

#include "ARDL/Model/Chain.hpp"
#include "ARDL/Util/Logger.hpp"
#include <chrono>

using namespace ARDL::Util;
using namespace ARDL::Model;

TEST_CASE("Deep Copy Check", "[model][copy]") {
    std::shared_ptr<Chain<double>> c;
#if !ARDL_EXTERNAL_DATA
    c= std::shared_ptr<Chain<double>>(new Chain<double>(urdfModel));
#else
    // Chain(std::string urdf_file, ARDL::Data<T> &data)
    ARDL::Data<double> data(urdfModel);
    // TODO NEEDED AS OTHERWISE REFERENCES ARE INVALIDATED! Possibly need to revert to in object storage or use
    // reserving for validation robustness or follow external data with no in object references
    c= std::shared_ptr<Chain<double>>(new Chain<double>(urdfModel, data));
#endif

    Eigen::VectorXd q, qd;
    q.resize(c->getNumOfJoints());
    qd.resize(c->getNumOfJoints());
    q.setZero();
    qd.setZero();
    q.setRandom();

    c->updateChain(q, qd);
    c->updateMatrices();

#if !ARDL_EXTERNAL_DATA
    std::shared_ptr<Chain<double>> c_copy;
    c_copy= std::shared_ptr<Chain<double>>(new Chain<double>(*c));
#else
    ARDL::Data<double> data2(data);
    // std::cout << "STARTING COPY" << std::endl;
    std::shared_ptr<Chain<double>> c_copy;
    c_copy= std::shared_ptr<Chain<double>>(new Chain<double>(*c, data2));
#endif

    // std::cout << "COPIED" << std::endl;
    checkApproxMatrix(c->getQ(), c_copy->getQ());

// #if !ARDL_EXTERNAL_DATA
//     checkApproxMatrix(c->getLinksRef().back()->getSI().calculateSpatialInertia(),
//                       c_copy->getLinksRef().back()->getSI().calculateSpatialInertia());
//     std::cout << ARDL_visit_ptr(c->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix() << std::endl;
//     checkApproxMatrix(ARDL_visit_ptr(c->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix(),
//                       ARDL_visit_ptr(c_copy->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix());
// #else
    checkApproxMatrix(c->getLinksRef().back().getSI().calculateSpatialInertia(),
                      c_copy->getLinksRef().back().getSI().calculateSpatialInertia());
    std::cout << ARDL_visit(c->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix() << std::endl;
    checkApproxMatrix(ARDL_visit(c->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix(),
                      ARDL_visit(c_copy->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix());
// #endif
    q(0)-= 0.5;
    qd(0)-= 0.2;
    q+= Eigen::VectorXd::Random(c->getNumOfJoints());
    qd+= Eigen::VectorXd::Random(c->getNumOfJoints());
    c->updateChain(q, qd);
    c->updateMatrices();
    REQUIRE(c->getQ() != c_copy->getQ());
    REQUIRE(c->getQd() != c_copy->getQd());
// #if !ARDL_EXTERNAL_DATA
// std::cout << ARDL_visit_ptr(c->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix() << std::endl
//               << std::endl;
//     std::cout << ARDL_visit_ptr(c_copy->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix() << std::endl
//               << std::endl;
//     REQUIRE(ARDL_visit_ptr(c->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix() !=
//             ARDL_visit_ptr(c_copy->getLinksRef().back()->getParentJoint(), getAdjointLocal()).getMatrix());
//     checkApproxMatrix(c->getLinksRef().back()->getSI().calculateSpatialInertia(),
//                       c_copy->getLinksRef().back()->getSI().calculateSpatialInertia());
// #else
    std::cout << ARDL_visit(c->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix() << std::endl
              << std::endl;
    std::cout << ARDL_visit(c_copy->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix() << std::endl
              << std::endl;
    REQUIRE(ARDL_visit(c->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix() !=
            ARDL_visit(c_copy->getLinksRef().back().getParentJoint(), getAdjointLocal()).getMatrix());
    checkApproxMatrix(c->getLinksRef().back().getSI().calculateSpatialInertia(),
                      c_copy->getLinksRef().back().getSI().calculateSpatialInertia());
// #endif
}

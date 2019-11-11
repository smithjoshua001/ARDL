#pragma once

#include "robotTestMain.cpp"
#include "Kinematics/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include <chrono>

using namespace ARDL::Util;

TEST_CASE("Deep Copy Check", "[model][copy]") {
    std::shared_ptr<Chain<double> > c;
    c = std::shared_ptr<Chain<double> >(new Chain<double>(urdfModel));

    Eigen::VectorXd q, qd;
    q.resize(c->getNumOfJoints());
    qd.resize(c->getNumOfJoints());
    q.setZero();
    qd.setZero();
    q.setRandom();

    c->updateChain(q, qd);
    c->updateMatrices();
    std::cout << "STARTING COPY" << std::endl;
    std::shared_ptr<Chain<double> > c_copy;
    c_copy = std::shared_ptr<Chain<double> >(new Chain<double>(*c));

    std::cout << "COPIED" << std::endl;
    checkApproxMatrix(c->getQ(), c_copy->getQ());
    q(0) -= 0.5;
    qd(0) -= 0.2;
    c->updateChain(q, qd);
    c->updateMatrices();
    REQUIRE(c->getQ() != c_copy->getQ());
    std::cout << c->getQ() << std::endl;
    std::cout << c_copy->getQ() << std::endl;
    REQUIRE(c->getQd() != c_copy->getQd());
}

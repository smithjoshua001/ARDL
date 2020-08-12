#pragma once

#include <ARDL/Kinematics/ForwardKinematics.hpp>
#include <ARDL/Dynamics/Dynamics.hpp>

#include <kdl/kdl.hpp>
#include <kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
//#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
    #error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace ARDL;
using namespace ARDL::Math;
using namespace ARDL::Model;

RigidBodyDynamics::Model setupRBDL(std::string filename) {
    RigidBodyDynamics::Model modelRBDL;
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), &modelRBDL, false, false)) {
        FAIL("Error loading modelRBDL");
    }
    return modelRBDL;
}

void setupKDL(std::string filename, KDL::Chain &chain_KDL, std::shared_ptr<KDL::ChainFkSolverPos_recursive> &jtc, std::shared_ptr<KDL::ChainJntToJacDotSolver> &jtjd, std::shared_ptr<KDL::ChainJntToJacSolver> &jtj, std::string rootName, std::string tipName) {
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(filename);
    KDL::Tree model_KDL;
    if (!kdl_parser::treeFromUrdfModel(*model, model_KDL)) {
        FAIL("Error loading model_KDL");
    }
    // "link0", "link3"
    model_KDL.getChain(rootName, tipName, chain_KDL);
    jtc = std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain_KDL));
    jtjd = std::shared_ptr<KDL::ChainJntToJacDotSolver>(new KDL::ChainJntToJacDotSolver(chain_KDL));
    jtj = std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain_KDL));
}

void setupPinocchio(std::string filename, pinocchio::Model &model, pinocchio::Data &data) {
    pinocchio::urdf::buildModel(filename, model);
    data = pinocchio::Data(model);
}

void setupARDL(std::string filename, std::shared_ptr<Chain<double> > &chain, std::shared_ptr<ForwardKinematics<double> > &fk) {
    chain = std::shared_ptr<Chain<double> >(new Chain<double>(filename));
    fk = std::shared_ptr<ForwardKinematics<double> >(new ForwardKinematics<double>(chain));
}

template <typename Derived, typename Dervied2> void inline checkApproxMatrix(const Eigen::MatrixBase<Derived> &first, const Eigen::MatrixBase<Dervied2> &second, double margin = 0.0, double eps = std::numeric_limits<float>::epsilon() *100) {
    for (int i = 0; i < first.size() - 1; i++) {
        CHECK(first(i) == Approx(second(i)).margin(margin).epsilon(eps));
    }
    REQUIRE(first(first.size() - 1) == Approx(second(first.size() - 1)).margin(margin).epsilon(eps));
}
void randomJointState(Eigen::VectorXd &q, Eigen::VectorXd &qd, Eigen::VectorXd &qdd) {
    for (int i = 0; i < q.size(); i++) {
        q(i) = (((double)rand() / (RAND_MAX)) - 0.5) * M_PI;
        qd(i) = (((double)rand() / (RAND_MAX)) - 0.5) * (M_PI) / 2;
        qdd(i) = (((double)rand() / (RAND_MAX)) - 0.5) * (M_PI / 3);
    }
    LOG_DEBUG_LEVEL3("Q: {}\n QD: {}\n QDD:{}", q.transpose(), qd.transpose(), qdd.transpose());
}

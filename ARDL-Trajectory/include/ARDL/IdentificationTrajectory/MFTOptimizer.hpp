#pragma once

#include <motion-generators/ModifiedFourierTrajectory.hpp>

#include <string>

#include <pagmo/io.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>
#include <pagmo/detail/visibility.hpp>
#include <pagmo/utils/gradients_and_hessians.hpp>

#include "ARDL/Model/Chain.hpp"
#include "ARDL/Dynamics/Dynamics.hpp"

#include <rapidjson/document.h>

using namespace rapidjson;

using namespace pagmo;
using namespace ARDL::Model;
using namespace ARDL;
using namespace ARDL::Math;
class PAGMO_DLL_PUBLIC MFTOptimizer {
public:
    std::string robotUrdf, configJson;
    void init() {
        init(robotUrdf, configJson);
    }
    void init(std::string roboturdf, std::string configJson) {
        this->robotUrdf = roboturdf;
        this->configJson = configJson;
        robot = std::shared_ptr<Chain<double> >(new Chain<double>(roboturdf));
        kin = std::shared_ptr<ForwardKinematics<double> >(new ForwardKinematics<double>(robot));
        dyn = std::shared_ptr<Dynamics<double> >(new Dynamics<double>(robot));
        dyn->calcBaseProjection(50000, 2.5, 1e-3);
        regressorProjector = dyn->getRegressorProjector();

        dof = robot->getNumOfJoints();
        configRobot = std::make_shared<rapidjson::Document>();
        std::ifstream ifs(configJson);
        IStreamWrapper isw(ifs);
        configRobot->ParseStream(isw);
        // std::cout << configJson << std::endl;
        if (!configRobot->IsObject()) {
            std::cout << "Not an object?" << std::endl;
            exit(-2);
        }
        // std::cout << "D1 " << (*configRobot)["trajectory_dof"].GetUint() << " " << robot->getNumOfJoints() << std::endl;
        MFT = std::shared_ptr<ModifiedFourierTrajectory<double> >(new ModifiedFourierTrajectory<double>(robot->getNumOfJoints(), (*configRobot)["trajectory_dof"].GetUint()));
        MFT->setGlobalPulsation((*configRobot)["wf_init"].GetDouble());
        samples = int(std::ceil(MFT->getPeriodLength() * (*configRobot)["controlRate"].GetDouble()));
        q.resize(dof);
        qd.resize(dof);
        qdd.resize(dof);
        FK.resize(samples, 3);
        linkPoses.resize(samples, 3 * robot->getNumOfLinks());
        regressor.resize(dof * samples, dyn->getNumOfBaseParams());
        trajectory_params.resize(2 * robot->getNumOfJoints() * (*configRobot)["trajectory_dof"].GetUint() + 1);
        trajectory_params.setZero();
        trajectory_params[0] = (*configRobot)["wf_init"].GetDouble();
        traj_dof = (*configRobot)["trajectory_dof"].GetUint();
        vars = robot->getNumOfJoints() * 2 * traj_dof;

        adjoints.resize(dof);
        linkAdjoints.resize(robot->getNumOfLinks() + 1);
        lbSE3.resize(dof);
        jacobians.resize(dof);
        jacobianDots.resize(dof);
        for (int i = 0; i < dof; i++) {
            jacobians.at(i).resize(6, dof);
            jacobians.at(i).setZero();
            jacobianDots.at(i).resize(6, dof);
            jacobianDots.at(i).setZero();
        }
        t_regressor.resize(dof, dof * 12);
        rrt.resize(dyn->getNumOfBaseParams(), dyn->getNumOfBaseParams());
        saes = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(dyn->getNumOfBaseParams());
        amax = bmax = (*configRobot)["trajectoryCoeffMax"].GetDouble();
        amin = bmin = (*configRobot)["trajectoryCoeffMin"].GetDouble();
        // std::cout << dof << std::endl;
        for (size_t i = 0; i < robot->getNumOfJoints() * traj_dof; i++) {
            boundsTmp.first.push_back(amin);
            boundsTmp.second.push_back(amax);
        }
        for (size_t i = 0; i < robot->getNumOfJoints() * traj_dof; i++) {
            boundsTmp.first.push_back(bmin);
            boundsTmp.second.push_back(bmax);
        }
        collision = jointLimit = velLimit = accLimit = linkLimit = false;
        iConNum = 0;
        if (configRobot->HasMember("collision_constraints")) {
            collision = true;
            constraintIndexes.push_back(iConNum);
            iConNum++;
        }
        if (configRobot->HasMember("joint_limit_constraint")) {
            jointLimit = true;
            constraintIndexes.push_back(iConNum);
            iConNum += dof * 2;
            jointLimitVec.resize(dof * 2);
            jointLimitRef.resize(dof * 2);
            const Value &min = (*configRobot)["joint_limit_constraint"]["min"];
            if (min.IsArray()) {
                for (size_t i = 0; i < min.Size(); i++) {
                    jointLimitRef[i] = min[i].GetDouble();
                }
            } else if (min.IsString()) {
                std::vector<std::pair<double, double> > limits;
                limits.resize(dof);
                robot->getJointLimits(limits);
                for (size_t i = 0; i < dof; i++) {
                    jointLimitRef[i] = limits[i].first;
                }
            }
            const Value &max = (*configRobot)["joint_limit_constraint"]["max"];
            if (max.IsArray()) {
                for (size_t i = 0; i < max.Size(); i++) {
                    jointLimitRef[i + dof] = max[i].GetDouble();
                }
            } else if (max.IsString()) {
                std::vector<std::pair<double, double> > limits;
                limits.resize(dof);
                robot->getJointLimits(limits);
                for (size_t i = 0; i < dof; i++) {
                    jointLimitRef[i + dof] = limits[i].second;
                }
            }
        }
        if (configRobot->HasMember("velocity_limit_constraint")) {
            velLimit = true;
            constraintIndexes.push_back(iConNum);
            iConNum += dof;

            velLimitVec.resize(dof);
            velLimitRef.resize(dof);
            const Value &limit = (*configRobot)["velocity_limit_constraint"];
            if (limit.IsArray()) {
                for (size_t i = 0; i < dof; i++) {
                    velLimitRef[i] = limit[i].GetDouble();
                }
            } else if (limit.IsDouble()) {
                velLimitRef.setConstant(limit.GetDouble());
            }
        }
        if (configRobot->HasMember("acc_limit_constraint")) {
            accLimit = true;
            constraintIndexes.push_back(iConNum);
            iConNum += dof;
            accLimitVec.resize(dof);
            accLimitRef.resize(dof);
            const Value &limit = (*configRobot)["acc_limit_constraint"];
            if (limit.IsArray()) {
                for (size_t i = 0; i < dof; i++) {
                    accLimitRef[i] = limit[i].GetDouble();
                }
            } else if (limit.IsDouble()) {
                accLimitRef.setConstant(limit.GetDouble());
            }
        }
        // if (configRobot->HasMember("pose_limit_constraints")) {
        //     poseLimit = true;accLimit
        //     constraintIndexes.push_back(iConNum);
        //     iConNum += 2 * 3;
        // }
        if (configRobot->HasMember("link_limit_constraints")) {
            linkLimit = true;
            linkLimitMin = Eigen::MatrixXd::Zero(3, robot->getNumOfLinks());
            linkLimitRef = Eigen::MatrixXd::Zero(3, robot->getNumOfLinks());
            const Value &limit = (*configRobot)["link_limit_constraints"];
            if (limit.IsArray()) {
                if (limit[0].IsArray() && limit.Size() == robot->getNumOfLinks()) {
                    for (size_t i = 0; i < robot->getNumOfLinks(); i++) {
                        linkLimitRef.col(i) << limit[i][0].GetDouble(), limit[i][1].GetDouble(), limit[i][2].GetDouble();
                        linkLimitMin.col(i) << limit[i][3].GetDouble(), limit[i][4].GetDouble(), limit[i][5].GetDouble();
                    }
                } else {
                    std::cerr << "Need to be array : " << robot->getNumOfLinks() << std::endl;
                    exit(-2);
                }
            } else {
                std::cerr << "Need to be array : " << robot->getNumOfLinks() << std::endl;
                exit(-2);
            }
            constraintIndexes.push_back(iConNum);
            iConNum += 2 * 3 * robot->getNumOfLinks();
        }
        fitness_output.resize(1 + iConNum);
    }
    MFTOptimizer(std::string roboturdf = "", std::string configjson = "") : robotUrdf(roboturdf), configJson(configjson) {
        if (roboturdf != "" || configjson != "") {
            init(roboturdf, configJson);
        }
    }
    MFTOptimizer(const MFTOptimizer &input) {
        MFT = std::shared_ptr<ModifiedFourierTrajectory<double> >(new ModifiedFourierTrajectory<double>(*(input.MFT)));
        robot = std::shared_ptr<Chain<double> >(new Chain<double>(*(input.robot)));
        dyn = std::shared_ptr<Dynamics<double> >(new Dynamics<double>(*(input.dyn), robot));
        kin = std::shared_ptr<ForwardKinematics<double> >(new ForwardKinematics<double>(*(input.kin), robot));
        robotUrdf = input.robotUrdf;
        configJson = input.configJson;
        // configRobot = std::shared_ptr<rapidjson::Document>(new rapidjson::Document(*(input.configRobot)));
        configRobot = std::shared_ptr<rapidjson::Document>(new rapidjson::Document());
        configRobot->CopyFrom(*(input.configRobot), configRobot->GetAllocator());
        trajectory_params = input.trajectory_params;
        samples = input.samples;
        dof = input.dof;
        traj_dof = input.traj_dof;
        vars = input.vars;
        q = input.q;
        qd = input.qd;
        qdd = input.qdd;
        FK = input.FK;
        linkPoses = input.linkPoses;
        regressor = input.regressor;
        regressorProjector = input.regressorProjector;
        rrt = input.rrt;
        adjoints = input.adjoints;
        linkAdjoints = input.linkAdjoints;
        lbSE3 = input.lbSE3;
        jacobians = input.jacobians;
        jacobianDots = input.jacobianDots;
        t_regressor = input.t_regressor;
        saes = input.saes;
        tmp = input.tmp;
        amin = input.amin;
        bmin = input.bmin;
        amax = input.amax;
        bmax = input.bmax;
        boundsTmp = input.boundsTmp;
        iConNum = input.iConNum;
        collision = input.collision;
        jointLimit = input.jointLimit;
        velLimit = input.velLimit;
        accLimit = input.accLimit;
        linkLimit = input.linkLimit;
        constraintIndexes = input.constraintIndexes;
        fitness_output = input.fitness_output;
        jointLimitVec = input.jointLimitVec;
        jointLimitRef = input.jointLimitRef;
        velLimitVec = input.velLimitVec;
        velLimitRef = input.velLimitRef;
        accLimitVec = input.accLimitVec;
        accLimitRef = input.accLimitRef;
        linkLimitMin = input.linkLimitMin;
        linkLimitRef = input.linkLimitRef;
    }
    // MFTOptimizer() {init(robotUrdf, configJson);};
    ~MFTOptimizer() {}
    // #include <chrono>

    vector_double gradient(const vector_double &x) const {
        return estimate_gradient_h([this](const vector_double &x) {return this->fitness(x);}, x);
    }

    // Mandatory, computes ... well ... the fitness
    vector_double fitness(const vector_double &x) const {
        if (robotUrdf == "" || configJson == "") {
            std::cerr << "NO URDF OR CONFIGURATION JSON" << std::endl;
            exit(-1);
        }
        // using namespace std::chrono;
        // steady_clock::time_point t1, t2;

        tmp = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(x.data(), vars);
        trajectory_params.tail(vars) = tmp;
        MFT->setFromParameters(trajectory_params);
        simulated = MFT->simulate((*configRobot)["controlRate"].GetDouble());

        bool collisionTmp = false;
        // std::cout << "COMPUTING!!" << std::endl;
        // std::cout << input->pos.cols() << std::endl;
        for (size_t i = 0; i < simulated->pos.cols(); i++) {
            // t1 = steady_clock::now();
            // int i = 4;
            q = simulated->pos.col(i);//.transpose();
            qd = simulated->vel.col(i);//.transpose();
            qdd = simulated->acc.col(i);//.transpose();

            robot->updateChain(q, qd);
            robot->updateMatrices();
            kin->getAdjoints(adjoints, true);
            if (collision && !collisionTmp) {
                collisionTmp = kin->collision();
            }
            kin->getBodyJacobian(jacobians, jacobianDots);
            kin->getLieBrackets(lbSE3, jacobians);
            dyn->calcSlotineLiRegressor(jacobians, jacobianDots, lbSE3, adjoints, qd, qdd, t_regressor);
            regressor.block(i * dof, 0, dof, dyn->getNumOfBaseParams()) = t_regressor * regressorProjector;

            FK.row(i) = adjoints.back().getP();
            if (linkLimit) {
                kin->getLinkAdjoints(linkAdjoints);
                for (size_t i1 = 0; i1 < robot->getNumOfLinks(); i1++) {
                    linkPoses.block<1, 3>(i, i1 * 3) = linkAdjoints[i1 + 1].getP();
                }
            }
            // t2 = steady_clock::now();
            // duration<double> time_span = duration_cast<duration<double> >(t2 - t1);

            // std::cout << "It took ARDL " << time_span.count() << " seconds.";
            // std::cout << std::endl;
        }

        rrt = regressor.transpose() * regressor;
        saes.compute(rrt);
        // std::cout << "chekgin?" << std::endl;
        //constraints
        if (jointLimit) {
            jointLimitVec.head(dof) = jointLimitRef.head(dof) - simulated->pos.rowwise().minCoeff();
            jointLimitVec.tail(dof) = simulated->pos.rowwise().maxCoeff() - jointLimitRef.tail(dof);
        }
        if (velLimit) {
            velLimitVec = simulated->vel.array().abs().rowwise().maxCoeff().matrix() - velLimitRef;
            // std::cout << velLimitVec << std::endl;
        }
        if (accLimit) {
            accLimitVec = simulated->acc.array().abs().rowwise().maxCoeff().matrix() - accLimitRef;
            // std::cout << velLimitVec << std::endl;
        }
        if (saes.eigenvalues()[0] > 0) {
            fitness_output[0] = saes.eigenvalues()[saes.eigenvalues().size() - 1] / saes.eigenvalues()[0];
        } else {
            fitness_output[0] = 1e100;
        }
        size_t constraint = 0;
        if (collision) {
            fitness_output[constraintIndexes[constraint++] + 1] = collisionTmp;
        }
        if (jointLimit) {
            for (size_t i = 0; i < dof * 2; i++) {
                fitness_output[constraintIndexes[constraint] + i + 1] = jointLimitVec[i];
            }
            constraint++;
        }
        if (velLimit) {
            for (size_t i = 0; i < dof; i++) {
                fitness_output[constraintIndexes[constraint] + i + 1] = velLimitVec[i];
            }
            constraint++;
        }
        if (accLimit) {
            for (size_t i = 0; i < dof; i++) {
                fitness_output[constraintIndexes[constraint] + i + 1] = accLimitVec[i];
            }
            constraint++;
        }
        if (linkLimit) {
            // linkPoses.resize(samples, 3 * dof);

            // linkLimitRef = Eigen::MatrixXd::Zero(3, robot->getNumOfLinks());
            for (size_t i = 0; i < robot->getNumOfLinks(); i++) {
                fitness_output[constraintIndexes[constraint] + i * 6 + 1] = linkPoses.col(i * 3).maxCoeff() - linkLimitRef(0, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 2] = linkPoses.col(i * 3 + 1).maxCoeff() - linkLimitRef(1, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 3] = linkPoses.col(i * 3 + 2).maxCoeff() - linkLimitRef(2, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 4] = linkLimitMin(0, i) - linkPoses.col(i * 3).minCoeff();
                fitness_output[constraintIndexes[constraint] + i * 6 + 5] = linkLimitMin(1, i) - linkPoses.col(i * 3 + 1).minCoeff();
                fitness_output[constraintIndexes[constraint] + i * 6 + 6] = linkLimitMin(2, i) - linkPoses.col(i * 3 + 2).minCoeff();
            }
            constraint++;
        }
        // return {saes.eigenvalues()[saes.eigenvalues().size() - 1] / saes.eigenvalues()[0]};
        return fitness_output;
    }

    // Mandatory, returns the box-bounds
    std::pair<vector_double, vector_double> get_bounds() const {
        return boundsTmp;
        // return {{0, 0, 0, 0}, {0, 0, 0, 0}};
    }

    vector_double::size_type get_nic() const {
        return iConNum;
    }

    // Optional, provides a name for the problem overrding the default name
    std::string get_name() const {
        return "MFT Trajecotry Optimization";
    }

    // Optional, provides extra information that will be appended after
    // the default stream operator
    std::string get_extra_info() const {
        return "This is a simple toy stochastic problem with one objective, no constraints and a fixed dimension of 8.";
    }
    thread_safety get_thread_safety() const {
        return thread_safety::none;
    }
    template <typename Archive> void serialize(Archive &ar, unsigned) {
        // ar &m_prob_id;
        detail::archive(ar, robotUrdf, configJson);
        // std::cout << robotUrdf << " SERIALIZED" << std::endl;
        init(robotUrdf, configJson);
    }

    std::shared_ptr<ModifiedFourierTrajectory<double> > getMFT() {
        return MFT;
    }

private:
    mutable std::shared_ptr<ModifiedFourierTrajectory<double> > MFT;
    mutable std::shared_ptr<Chain<double> > robot;
    mutable std::shared_ptr<Dynamics<double> > dyn;
    mutable std::shared_ptr<ForwardKinematics<double> > kin;
    mutable std::shared_ptr<rapidjson::Document> configRobot;
    mutable Eigen::VectorXd trajectory_params;
    mutable size_t samples, dof, traj_dof, vars;
    mutable Eigen::VectorXd q, qd, qdd;
    mutable Eigen::MatrixXd FK, linkPoses, regressor, regressorProjector, rrt;
    mutable std::shared_ptr<TrajectoryStorage<double> > simulated;
    mutable aligned_vector<AdjointSE3<double> > adjoints, linkAdjoints;
    mutable aligned_vector<LieBracketSE3<double> > lbSE3;
    mutable aligned_vector<Jacobian<double> > jacobians, jacobianDots;
    mutable Regressor<double> t_regressor;
    mutable Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes;
    mutable Eigen::VectorXd tmp;
    mutable double amin, bmin, amax, bmax;
    mutable std::pair<vector_double, vector_double> boundsTmp;
    size_t iConNum;
    bool collision, jointLimit, velLimit, accLimit, linkLimit;
    std::vector<size_t> constraintIndexes;

    mutable vector_double fitness_output;

    mutable Eigen::VectorXd jointLimitVec, jointLimitRef, velLimitVec, velLimitRef, accLimitVec, accLimitRef;
    mutable Eigen::MatrixXd linkLimitMin, linkLimitRef;
};

PAGMO_S11N_PROBLEM_EXPORT(MFTOptimizer)

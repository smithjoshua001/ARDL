#include <motion-generators/MultiModifiedFourierTrajectory.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include "ARDL/Model/Chain.hpp"
#include "ARDL/Dynamics/Dynamics.hpp"
#include "ARDL/Constraints/Constraints.hpp"
#include "ARDL/typedefs.hpp"
#include "ARDL/Util/ModelPath.hpp"
#include "ARDL/Util/BufferLogging.hpp"

#include <rapidjson/document.h>
using namespace rapidjson;
using namespace ARDL;
using namespace ARDL::Model;
using namespace ARDL::Constraints;

aligned_vector<AdjointSE3<double> > adjoints;

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage) {
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

std::vector < ConstraintsVariant<double> > constraints;
void initConstraints(const Document &constraintDoc, std::shared_ptr<Chain<double> > model) {
    std::shared_ptr<ForwardKinematics<double> > kin(new ForwardKinematics<double>(model));
    if (constraintDoc.HasMember("collision_constraints")) {
        constraints.push_back(SelfCollision<double>(model, kin));
    }
    if (constraintDoc.HasMember("joint_limit_constraint")) {
        constraints.push_back(JointPositionLimit<double>(model));
        const Value &min = constraintDoc["joint_limit_constraint"]["min"];
        Eigen::VectorXd tmp(model->getNumOfJoints());
        if (min.IsArray()) {
            for (size_t i = 0; i < min.Size(); i++) {
                tmp[i] = min[i].GetDouble();
            }
            std::get<JointPositionLimit<double> >(constraints.back()).setMin(tmp);
        }

        const Value &max = constraintDoc["joint_limit_constraint"]["max"];
        if (max.IsArray()) {
            for (size_t i = 0; i < min.Size(); i++) {
                tmp[i] = max[i].GetDouble();
            }
            std::get<JointPositionLimit<double> >(constraints.back()).setMax(tmp);
        }
    }
    if (constraintDoc.HasMember("velocity_limit_constraint")) {
        Eigen::VectorXd velLimit(model->getNumOfJoints());
        const Value &limit = constraintDoc["velocity_limit_constraint"];
        if (limit.IsArray()) {
            for (size_t i = 0; i < model->getNumOfJoints(); i++) {
                velLimit[i] = limit[i].GetDouble();
            }
        } else if (limit.IsDouble()) {
            velLimit.setConstant(limit.GetDouble());
        }
        constraints.push_back(JointVelocityLimit<double>(model, velLimit));
    }
    if (constraintDoc.HasMember("link_limit_constraints")) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> max, min;
        max.resize(3, model->getNumOfJoints()+1);
        min.resize(3, model->getNumOfJoints()+1);

        const Value &limit = constraintDoc["link_limit_constraints"];
        if (limit.IsArray()) {
            if (limit[0].IsArray() && limit.Size() == model->getNumOfJoints()+1) {
                for (size_t i = 0; i < model->getNumOfJoints()+1; i++) {
                    max.col(i) << limit[i][0].GetDouble(), limit[i][1].GetDouble(), limit[i][2].GetDouble();
                    min.col(i) << limit[i][3].GetDouble(), limit[i][4].GetDouble(), limit[i][5].GetDouble();
                }
            } else {
                std::cerr << "Need to be array : " << model->getNumOfJoints()+1 << std::endl;
                exit(-2);
            }
        } else {
            std::cerr << "Need to be array : " << model->getNumOfJoints()+1 << std::endl;
            exit(-2);
        }
        constraints.push_back(CartesianLink<double>(model, adjoints, min, max));
    }
}

// template<class ... Ts> struct overload : Ts ... {
//     using Ts::operator() ...;
// };
// template<class ... Ts> overload(Ts ...)->overload<Ts...>;
#include "rapidjson/stringbuffer.h"
#include <rapidjson/writer.h>

int main(int argc, char **argv) {
    std::string urdf = "", config = "", output="";
    if (argc > 4) {
        urdf = argv[1];
        config = argv[2];
        output = argv[3];
    } else {
        std::cout << "NO URDF CONFIG OR 3ARG" << std::endl;
        return -3;
    }

    urdf= ARDL::Util::getModelFromGazeboPath(urdf);
    Document constraintsDoc;
    std::ifstream ifs(config);
    IStreamWrapper isw(ifs);
    constraintsDoc.ParseStream(isw);
    if (!constraintsDoc.IsObject()) {
        return -1;
    }
    std::shared_ptr<Chain<double> > model(new Chain<double>(urdf));
    std::shared_ptr<ForwardKinematics<double> > kin(new ForwardKinematics<double>(model));
    std::shared_ptr<Dynamics<double> > dyn = std::shared_ptr<Dynamics<double> >(new Dynamics<double>(model));

    aligned_vector<LieBracketSE3<double> > lbSE3;
    aligned_vector<Jacobian<double> > jacobians, jacobianDots;
    size_t dof = model->getNumOfJoints();
    lbSE3.resize(dof);
    jacobians.resize(dof);
    jacobianDots.resize(dof);
    for (int i = 0; i < dof; i++) {
        jacobians.at(i).resize(dof);
        jacobians.at(i).setZero();
        jacobianDots.at(i).resize(dof);
        jacobianDots.at(i).setZero();
    }

    adjoints.resize(model->getNumOfJoints()+1);
    Regressor<double> t_regressor;
    t_regressor.resize(dof);

    Eigen::VectorXd params(model->getNumOfJoints()*10);
    model->getParams(params);

    initConstraints(constraintsDoc, model);

    Eigen::VectorXd q(model->getNumOfJoints()), qd(model->getNumOfJoints()), qdd(model->getNumOfJoints()), tau(model->getNumOfJoints());
    q.setZero(); qd.setZero();

    std::vector<std::string> configFiles;

    for (size_t k = 0; k < argc - 4; k++) {
        configFiles.push_back(std::string(argv[4 + k]));
    }

    std::shared_ptr<MultiModifiedFourierTrajectory<double> > MFT = std::shared_ptr<MultiModifiedFourierTrajectory<double> >(new MultiModifiedFourierTrajectory<double>(configFiles, dof));
    std::shared_ptr<TrajectoryStorage<double> > simulated;

    size_t samples = int(std::ceil(MFT->getPeriodLength() * 100));// constraintsDoc["controlRate"].GetDouble()));
    
    simulated = MFT->simulate(100);//MFT->simulate(constraintsDoc["controlRate"].GetDouble());


    ARDL::Util::BufferLogging::CSV log(output, 200);


    log.addField("q_{}", dof);
    log.addField("qd_{}", dof);
    log.addField("qdd_{}", dof);
    log.addField("tau_{}", dof);
    log.saveFields();

    Eigen::MatrixXd linkPoses(samples, 3 * model->getNumOfLinks());
    bool valid = true;
    for (size_t i = 0; i < simulated->pos.cols(); i++) {
        model->updateChain(simulated->pos.col(i), simulated->vel.col(i));
        model->updateMatricesOptim();
        kin->getBodyAdjointsOptim(adjoints,true);
        kin->getJacobians<ARDL::Frame::BODY>(adjoints,jacobians);
        kin->getLieBrackets(lbSE3, jacobians);
        dyn->calcSlotineLiRegressor(qd, simulated->acc.col(i), adjoints, lbSE3, jacobians, jacobianDots, t_regressor);
        tau = t_regressor*params;

        for (size_t j = 0; j < constraints.size(); j++) {
            valid &= !std::visit([](auto &constraint) -> bool {return constraint.compute();}, constraints[j]);
        }
        if (!valid) {
            std::cout << "Not Valid " << std::endl;
            MFT->display();
            std::cout << "failed on step: " << i << std::endl;
            for (size_t j = 0; j < constraints.size(); j++) {
                std::cout << std::visit([](auto &constraint) -> VectorX<double> {return constraint.getConstraintVector();}, constraints[j]).transpose() << std::endl;
            }
            break;
        }
        q =  simulated->pos.col(i);
        qd =  simulated->vel.col(i);
        qdd =  simulated->acc.col(i);
        log.addData("q_{}",q);
        log.addData("qd_{}", qd);
        log.addData("qdd_{}", qdd);
        log.addData("tau_{}", tau);
        log.saveAndClear();
        printProgress(double(i) / double(simulated->pos.cols() - 1));
    }
    log.save();
    log.clear();
    log.close();
}

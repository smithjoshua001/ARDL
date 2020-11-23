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

namespace Eigen{
template<class Matrix>
void write_binary(const char* filename, const Matrix& matrix){
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
}
template<class Matrix>
void read_binary(const char* filename, Matrix& matrix){
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
}
} // Eigen::

class PAGMO_DLL_PUBLIC MFTOptimizer {
     public:
    int copies= 0;
    void init() { init(robotUrdf, configJson); }
    void init(std::string roboturdf, std::string configJson) {
        std::cout<<"INITING!!"<<std::endl;
        this->robotUrdf= roboturdf;
        this->configJson= configJson;

        configRobot= std::make_shared<rapidjson::Document>();
        std::ifstream ifs(configJson);
        IStreamWrapper isw(ifs);
        configRobot->ParseStream(isw);
        if(!configRobot->IsObject()) {
            std::cout << "Not an object?" << std::endl;
            exit(-2);
        }

        traj_dof= (*configRobot)["trajectory_dof"].GetUint();

        robot= std::shared_ptr<Chain<double>>(new Chain<double>(roboturdf));
        kin= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(robot));
        dyn= std::shared_ptr<Dynamics<double>>(new Dynamics<double>(robot));
        if(ifstream("regressorMatrix.dat").good()){
            Eigen::read_binary("regressorMatrix.dat",regressorProjector);
            dyn->setRegressorProjector(regressorProjector);
        }else{
        dyn->calcBaseProjection(50000, 5, 1e-10);
        regressorProjector= dyn->getRegressorProjector();
        }
        dof= robot->getNumOfJoints();

        MFT= std::shared_ptr<ModifiedFourierTrajectory<double>>(
            new ModifiedFourierTrajectory<double>(robot->getNumOfJoints(), (*configRobot)["trajectory_dof"].GetUint()));
        MFT->setGlobalPulsation((*configRobot)["wf_init"].GetDouble());

Eigen::VectorXd q(dof);
        if(configRobot->HasMember("startQ")) {
            for(size_t i= 0; i < robot->getNumOfJoints(); i++) { q[i]= (*configRobot)["startQ"][i].GetDouble(); }
            MFT->setQinit(q);
        }


        amax= bmax= (*configRobot)["trajectoryCoeffMax"].GetDouble();
        amin= bmin= (*configRobot)["trajectoryCoeffMin"].GetDouble();

        for(size_t i= 0; i < robot->getNumOfJoints() * traj_dof; i++) {
            boundsTmp.first.push_back(amin);
            boundsTmp.second.push_back(amax);
        }
        for(size_t i= 0; i < robot->getNumOfJoints() * traj_dof; i++) {
            boundsTmp.first.push_back(bmin);
            boundsTmp.second.push_back(bmax);
        }
        collision= jointLimit= velLimit= accLimit= linkLimit= false;
        iConNum= 0;
        if(configRobot->HasMember("collision_constraints")) {
            collision= true;
            constraintIndexes.push_back(iConNum);
            iConNum++;
        }
        if(configRobot->HasMember("joint_limit_constraint")) {
            jointLimit= true;
            constraintIndexes.push_back(iConNum);
            iConNum+= dof * 2;
            jointLimitRef.resize(dof * 2);
            const Value &min= (*configRobot)["joint_limit_constraint"]["min"];
            if(min.IsArray()) {
                for(size_t i= 0; i < min.Size(); i++) { jointLimitRef[i]= min[i].GetDouble(); }
            } else if(min.IsString()) {
                std::vector<std::pair<double, double>> limits;
                limits.resize(dof);
                robot->getJointLimits(limits);
                for(size_t i= 0; i < dof; i++) { jointLimitRef[i]= limits[i].first; }
            }
            const Value &max= (*configRobot)["joint_limit_constraint"]["max"];
            if(max.IsArray()) {
                for(size_t i= 0; i < max.Size(); i++) { jointLimitRef[i + dof]= max[i].GetDouble(); }
            } else if(max.IsString()) {
                std::vector<std::pair<double, double>> limits;
                limits.resize(dof);
                robot->getJointLimits(limits);
                for(size_t i= 0; i < dof; i++) { jointLimitRef[i + dof]= limits[i].second; }
            }
            if((*configRobot)["joint_limit_constraint"].HasMember("scale")) {
                jointLimitRef*= (*configRobot)["joint_limit_constraint"]["scale"].GetDouble();
            }
        }
        if(configRobot->HasMember("velocity_limit_constraint")) {
            velLimit= true;
            constraintIndexes.push_back(iConNum);
            iConNum+= dof;
            velLimitRef.resize(dof);
            const Value &limit= (*configRobot)["velocity_limit_constraint"];
            if(limit.IsArray()) {
                for(size_t i= 0; i < dof; i++) { velLimitRef[i]= limit[i].GetDouble(); }
            } else if(limit.IsDouble()) {
                velLimitRef.setConstant(limit.GetDouble());
            } else if(limit.IsString()) {
                robot->getJointVelocityLimits(velLimitRef);
            }
            // if((*configRobot)["velocity_limit_constraint"].HasMember("scale")){
            //   velLimitRef*=(*configRobot)["velocity_limit_constraint"]["scale"].GetDouble();
            // }
        }
        if(configRobot->HasMember("acc_limit_constraint")) {
            accLimit= true;
            constraintIndexes.push_back(iConNum);
            iConNum+= dof;
            accLimitRef.resize(dof);
            const Value &limit= (*configRobot)["acc_limit_constraint"];
            if(limit.IsArray()) {
                for(size_t i= 0; i < dof; i++) { accLimitRef[i]= limit[i].GetDouble(); }
            } else if(limit.IsDouble()) {
                accLimitRef.setConstant(limit.GetDouble());
            }
        }
        if(configRobot->HasMember("link_limit_constraints")) {
            linkLimit= true;
            linkLimitMin= Eigen::MatrixXd::Zero(3, robot->getNumOfJoints() + 1);
            linkLimitRef= Eigen::MatrixXd::Zero(3, robot->getNumOfJoints() + 1);
            const Value &limit= (*configRobot)["link_limit_constraints"];
            if(limit.IsArray()) {
                if(limit[0].IsArray() && limit.Size() == robot->getNumOfJoints() + 1) {
                    for(size_t i= 0; i < robot->getNumOfJoints() + 1; i++) {
                        linkLimitRef.col(i) << limit[i][0].GetDouble(), limit[i][1].GetDouble(),
                            limit[i][2].GetDouble();
                        linkLimitMin.col(i) << limit[i][3].GetDouble(), limit[i][4].GetDouble(),
                            limit[i][5].GetDouble();
                    }
                } else {
                    std::cerr << "Need to be array : " << robot->getNumOfJoints() << std::endl;
                    exit(-2);
                }
            } else {
                std::cerr << "Need to be array : " << robot->getNumOfJoints() << std::endl;
                exit(-2);
            }
            constraintIndexes.push_back(iConNum);
            iConNum+= 2 * 3 * (robot->getNumOfJoints() + 1);
        }
    }
    MFTOptimizer(std::string roboturdf= "", std::string configjson= ""): robotUrdf(roboturdf), configJson(configjson) {
        if(roboturdf != "" || configjson != "") { init(roboturdf, configJson); }
    }
    MFTOptimizer(const MFTOptimizer &input) {
        std::cout << "MFT COPY" << std::endl;
        MFT= std::shared_ptr<ModifiedFourierTrajectory<double>>(new ModifiedFourierTrajectory<double>(*(input.MFT)));
        robot= std::shared_ptr<Chain<double>>(new Chain<double>(*(input.robot)));
        dyn= std::shared_ptr<Dynamics<double>>(new Dynamics<double>(*(input.dyn), robot));
        kin= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(*(input.kin), robot));
        std::cout << "MFT COPY2" << std::endl;
        robotUrdf= input.robotUrdf;
        configJson= input.configJson;
        configRobot= std::shared_ptr<rapidjson::Document>(new rapidjson::Document());
        configRobot->CopyFrom(*(input.configRobot), configRobot->GetAllocator());
        dof= input.dof;
        traj_dof= input.traj_dof;
        regressorProjector= input.regressorProjector;
        amin= input.amin;
        std::cout << "MFT COPY4" << std::endl;
        bmin= input.bmin;
        amax= input.amax;
        bmax= input.bmax;
        boundsTmp= input.boundsTmp;
        iConNum= input.iConNum;
        collision= input.collision;
        jointLimit= input.jointLimit;
        velLimit= input.velLimit;
        accLimit= input.accLimit;
        linkLimit= input.linkLimit;
        constraintIndexes= input.constraintIndexes;
        std::cout << "MFT COPY5" << std::endl;
        jointLimitRef= input.jointLimitRef;
        velLimitRef= input.velLimitRef;
        accLimitRef= input.accLimitRef;
        linkLimitMin= input.linkLimitMin;
        linkLimitRef= input.linkLimitRef;
        std::cout << "MFT COPY6" << std::endl;
    }
    ~MFTOptimizer() {}
    // #include <chrono>

    vector_double gradient(const vector_double &x) const {
        return estimate_gradient_h([this](const vector_double &x) { return this->fitness(x); }, x);
    }

    // Mandatory, computes ... well ... the fitness
    vector_double fitness(const vector_double &x) const {
        // std::cout<<"fitness"<<std::endl;
        Eigen::VectorXd jointLimitVec, velLimitVec, accLimitVec;
        vector_double fitness_output;
        std::shared_ptr<ModifiedFourierTrajectory<double>> MFT;
        std::shared_ptr<Chain<double>> robot;
        std::shared_ptr<Dynamics<double>> dyn;
        std::shared_ptr<ForwardKinematics<double>> kin;
        std::shared_ptr<rapidjson::Document> configRobot;
        Eigen::VectorXd trajectory_params;
        size_t samples, vars;
        Eigen::VectorXd q, qd, qdd;
        Eigen::MatrixXd FK, linkPoses, regressor, rrt;
        std::shared_ptr<TrajectoryStorage<double>> simulated;
        aligned_vector<Pose<double>> adjoints, linkAdjoints;
        aligned_vector<Motion<double>> lbSE3;
        aligned_vector<Jacobian<double>> jacobians, jacobianDots;
        Regressor<double> t_regressor;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes;
        Eigen::VectorXd tmp;

        // std::cout<<"fitness1"<<std::endl;
        // MFT= std::shared_ptr<ModifiedFourierTrajectory<double>>(new ModifiedFourierTrajectory<double>(*(this->MFT)));
        // robot= std::shared_ptr<Chain<double>>(new Chain<double>(*(this->robot)));
        // dyn= std::shared_ptr<Dynamics<double>>(new Dynamics<double>(*(this->dyn), robot));
        // kin= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(*(this->kin), robot));

        configRobot= std::shared_ptr<rapidjson::Document>(new rapidjson::Document());
        configRobot->CopyFrom(*(this->configRobot), configRobot->GetAllocator());
          robot= std::shared_ptr<Chain<double>>(new Chain<double>(robotUrdf));
        kin= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(robot));
        dyn= std::shared_ptr<Dynamics<double>>(new Dynamics<double>(robot));

        MFT= std::shared_ptr<ModifiedFourierTrajectory<double>>(
            new ModifiedFourierTrajectory<double>(robot->getNumOfJoints(), (*configRobot)["trajectory_dof"].GetUint()));
        MFT->setGlobalPulsation((*configRobot)["wf_init"].GetDouble());

        samples= int(std::ceil(MFT->getPeriodLength() * (*configRobot)["controlRate"].GetDouble()));
        q.resize(dof);
        if(configRobot->HasMember("startQ")) {
            for(size_t i= 0; i < robot->getNumOfJoints(); i++) { q[i]= (*configRobot)["startQ"][i].GetDouble(); }
            MFT->setQinit(q);
        }

        // std::cout<<"fitness2"<<std::endl;
        // std::cout<<"DOF "<<dof<<std::endl;
        // std::cout<<"SAMPLES "<<samples<<std::endl;
        qd.resize(dof);
        qdd.resize(dof);
        FK.resize(samples, 3);
        // std::cout<<"fitness2.1"<<std::endl;
        linkPoses.resize(samples, 3 * (robot->getNumOfJoints() + 1));
        // regressor.resize(dof * samples, dyn->getNumOfBaseParams());
        regressor.resize(this->dyn->getNumOfBaseParams(), this->dyn->getNumOfBaseParams());
        // std::cout<<"fitness2.2"<<std::endl;
        trajectory_params.resize(2 * robot->getNumOfJoints() * (*configRobot)["trajectory_dof"].GetUint() + 1);
        trajectory_params.setZero();
        trajectory_params[0]= (*configRobot)["wf_init"].GetDouble();
        // std::cout<<"fitness2.3"<<std::endl;
        vars= robot->getNumOfJoints() * 2 * traj_dof;

        // std::cout<<"fitness2.4"<<std::endl;
        adjoints.resize(dof + 1);
        linkAdjoints.resize(robot->getNumOfJoints() + 1);
        lbSE3.resize(dof);
        jacobians.resize(dof);
        jacobianDots.resize(dof);
        // std::cout<<"fitness2.5"<<std::endl;
        for(int i= 0; i < dof; i++) {
            jacobians.at(i).resize(dof);
            jacobians.at(i).setZero();
            jacobianDots.at(i).resize(dof);
            jacobianDots.at(i).setZero();
        }
        // std::cout<<"fitness2.6"<<std::endl;
        t_regressor.resize(dof);
        rrt.resize(this->dyn->getNumOfBaseParams(), this->dyn->getNumOfBaseParams());
        saes= Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(this->dyn->getNumOfBaseParams());

        // std::cout<<"fitness3"<<std::endl;
        fitness_output.resize(1 + iConNum);
        jointLimitVec.resize(dof * 2);
        velLimitVec.resize(dof * 2);
        accLimitVec.resize(dof * 2);

        // std::cout<<"D1"<<std::endl;
        if(robotUrdf == "" || configJson == "") {
            std::cerr << "NO URDF OR CONFIGURATION JSON" << std::endl;
            exit(-1);
        }

        // std::cout<<"fitness4"<<std::endl;
        tmp= Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(x.data(), vars);
        trajectory_params.tail(vars)= tmp;
        MFT->setFromParameters(trajectory_params);
        simulated= MFT->simulate((*configRobot)["controlRate"].GetDouble());

        // std::cout<<"D2"<<std::endl;
        bool collisionTmp= false;
        regressor.setZero();
        // std::cout<<"fitness5"<<std::endl;
        for(size_t i= 0; i < simulated->pos.cols(); i++) {
            q= simulated->pos.col(i);   //.transpose();
            qd= simulated->vel.col(i);  //.transpose();
            qdd= simulated->acc.col(i); //.transpose();
            robot->updateChain(q, qd);
            robot->updateMatricesOptim();
            kin->getBodyAdjointsOptim(adjoints, true);
            if(collision && !collisionTmp) { collisionTmp= kin->collisionOptim(); }
            kin->getJacobians<Frame::BODY>(adjoints, jacobians);
            kin->getLieBrackets(lbSE3, jacobians);
            dyn->calcSlotineLiRegressor(qd, qdd, adjoints, lbSE3, jacobians, jacobianDots, t_regressor);
            // regressor.block(i * dof, 0, dof, dyn->getNumOfBaseParams()) = t_regressor * regressorProjector;
            regressor+= (t_regressor * regressorProjector).transpose() * (t_regressor * regressorProjector);

            kin->getAdjointsOptim(linkAdjoints, true);
            FK.row(i)= linkAdjoints.back().getP();
            if(linkLimit) {
                for(size_t i1= 0; i1 < robot->getNumOfJoints() + 1; i1++) {
                    linkPoses.block<1, 3>(i, i1 * 3)= linkAdjoints[i1].getP();
                }
            }
        }
        // std::cout<<"fitness6"<<std::endl;

        // std::cout<<"D3"<<std::endl;
        // rrt = regressor.transpose() * regressor;
        rrt= regressor;
        saes.compute(rrt);
        // std::cout<<"D4"<<std::endl;
        // std::cout<<"fitness7"<<std::endl;
        // constraints
        if(jointLimit) {
            jointLimitVec.head(dof)= jointLimitRef.head(dof) - simulated->pos.rowwise().minCoeff();
            jointLimitVec.tail(dof)= simulated->pos.rowwise().maxCoeff() - jointLimitRef.tail(dof);
        }
        // std::cout<<"D5"<<std::endl;
        if(velLimit) { velLimitVec= simulated->vel.array().abs().rowwise().maxCoeff().matrix() - velLimitRef; }
        // std::cout<<"D6"<<std::endl;
        if(accLimit) { accLimitVec= simulated->acc.array().abs().rowwise().maxCoeff().matrix() - accLimitRef; }
        // std::cout<<"D7"<<std::endl;
        // std::cout<<"EVs: "<<saes.eigenvalues().transpose()<<std::endl;
        if(saes.eigenvalues()[0] > 0) {
            // fitness_output[0] = saes.eigenvalues()[saes.eigenvalues().size() - 1] / saes.eigenvalues()[0];
            fitness_output[0]= -std::log(saes.eigenvalues().prod());
        } else {
            fitness_output[0]= 1e100;
        }
        // std::cout<<"D8"<<std::endl;
        size_t constraint= 0;
        // std::cout<<"D9"<<std::endl;
        if(collision) { fitness_output[constraintIndexes[constraint++] + 1]= collisionTmp; }
        // std::cout<<"D10"<<std::endl;
        if(jointLimit) {
            for(size_t i= 0; i < dof * 2; i++) {
                fitness_output[constraintIndexes[constraint] + i + 1]= jointLimitVec[i];
            }
            constraint++;
        }
        // std::cout<<"D11"<<std::endl;
        if(velLimit) {
            for(size_t i= 0; i < dof; i++) { fitness_output[constraintIndexes[constraint] + i + 1]= velLimitVec[i]; }
            constraint++;
        }
        // std::cout<<"D12"<<std::endl;
        if(accLimit) {
            for(size_t i= 0; i < dof; i++) { fitness_output[constraintIndexes[constraint] + i + 1]= accLimitVec[i]; }
            constraint++;
        }
        // std::cout<<"D13"<<std::endl;
        if(linkLimit) {
            for(size_t i= 0; i < robot->getNumOfJoints() + 1; i++) {
                fitness_output[constraintIndexes[constraint] + i * 6 + 1]=
                    linkPoses.col(i * 3).maxCoeff() - linkLimitRef(0, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 2]=
                    linkPoses.col(i * 3 + 1).maxCoeff() - linkLimitRef(1, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 3]=
                    linkPoses.col(i * 3 + 2).maxCoeff() - linkLimitRef(2, i);
                fitness_output[constraintIndexes[constraint] + i * 6 + 4]=
                    linkLimitMin(0, i) - linkPoses.col(i * 3).minCoeff();
                fitness_output[constraintIndexes[constraint] + i * 6 + 5]=
                    linkLimitMin(1, i) - linkPoses.col(i * 3 + 1).minCoeff();
                fitness_output[constraintIndexes[constraint] + i * 6 + 6]=
                    linkLimitMin(2, i) - linkPoses.col(i * 3 + 2).minCoeff();
            }
            constraint++;
        }
        // std::cout<<"fitness8"<<std::endl;
        return fitness_output;
    }

    // Mandatory, returns the box-bounds
    std::pair<vector_double, vector_double> get_bounds() const { return boundsTmp; }

    vector_double::size_type get_nic() const { return iConNum; }

    // Optional, provides a name for the problem overrding the default name
    std::string get_name() const { return "MFT Trajecotry Optimization"; }

    // Optional, provides extra information that will be appended after
    // the default stream operator
    std::string get_extra_info() const {
        return "This is a simple toy stochastic problem with one objective, no constraints and a fixed dimension of 8.";
    }
    thread_safety get_thread_safety() const {
        return thread_safety::constant;
        // return thread_safety::none;
    }
    template<typename Archive>
    void serialize(Archive &ar, unsigned) {
        // ar &m_prob_id;
        // detail::archive(ar);
        std::cout << robotUrdf << " SERIALIZED" << std::endl;
        detail::archive(ar, robotUrdf, configJson);
        // std::cout << robotUrdf << " SERIALIZED" << std::endl;
        init(robotUrdf, configJson);
    }

    std::shared_ptr<ModifiedFourierTrajectory<double>> getMFT() { return MFT; }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

     private:

    std::string robotUrdf, configJson;
    std::shared_ptr<ModifiedFourierTrajectory<double>> MFT;
    std::shared_ptr<Chain<double>> robot;
    std::shared_ptr<Dynamics<double>> dyn;
    std::shared_ptr<ForwardKinematics<double>> kin;
    std::shared_ptr<rapidjson::Document> configRobot;
    Eigen::MatrixXd regressorProjector;
    size_t dof, traj_dof;

    double amin, bmin, amax, bmax;
    std::pair<vector_double, vector_double> boundsTmp;
    size_t iConNum;
    bool collision, jointLimit, velLimit, accLimit, linkLimit;
    std::vector<size_t> constraintIndexes;

    Eigen::VectorXd jointLimitRef, velLimitRef, accLimitRef;
    Eigen::MatrixXd linkLimitMin, linkLimitRef;
};
PAGMO_S11N_PROBLEM_EXPORT(MFTOptimizer)

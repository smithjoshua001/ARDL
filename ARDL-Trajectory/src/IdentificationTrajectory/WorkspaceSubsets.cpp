#include "ARDL/Kinematics/ForwardKinematics.hpp"

#include "ARDL/Util/MatrixInitializer.hpp"
#include "ARDL/Util/Pseudoinverse.hpp"

#include <string>
#include <limits>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>

#include <rapidjson/prettywriter.h>
#include "ARDL/Util/ModelPath.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
using namespace rapidjson;
using namespace ARDL;
using namespace ARDL::Math;
using namespace ARDL::Model;

aligned_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> separateCube(const Eigen::Vector3d &min,
                                                                         const Eigen::Vector3d &max) {
    aligned_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> out;

    Eigen::Vector3d center= ((max + min) / 2.0);
    // cube back left lower
    out.push_back({min, center});

    // Cube back right lower
    Eigen::Vector3d rightCenter(center(0), max(1), center(2));
    Eigen::Vector3d lowerBackCenter(min(0), center(1), min(2));
    out.push_back({lowerBackCenter, rightCenter});

    // Cube forward right lower
    Eigen::Vector3d rightForwardCenter(max(0), max(1), center(2));
    Eigen::Vector3d lowerCenter(center(0), center(1), min(2));
    out.push_back({lowerCenter, rightForwardCenter});

    // cube forward left lower
    Eigen::Vector3d fowardCenter(max(0), center(1), center(2));
    Eigen::Vector3d leftLowerCenter(center(0), min(1), min(2));
    out.push_back({leftLowerCenter, fowardCenter});

    // cube forward left upper
    Eigen::Vector3d upperForwardCenter(max(0), center(1), max(2));
    Eigen::Vector3d leftCenter(center(0), min(1), center(2));
    out.push_back({leftCenter, upperForwardCenter});

    // cube backward right upper
    Eigen::Vector3d rightUpperCenter(center(0), max(1), max(2));
    Eigen::Vector3d backwardCenter(min(0), center(1), center(2));
    out.push_back({backwardCenter, rightUpperCenter});

    // cube backward left upper
    Eigen::Vector3d upperCenter(center(0), center(1), max(2));
    Eigen::Vector3d leftBackCenter(min(0), min(1), center(2));
    out.push_back({leftBackCenter, upperCenter});

    // cube forward right upper
    out.push_back({center, max});
    return out;
}
int main(int argc, char **argv) {
    size_t samples= 500000;
    std::string urdf, templateJson, outputPath;
    double shiftscale= 0.05;
    if(argc > 4) {
        samples= std::atoi(argv[1]);
        urdf= std::string(argv[2]);
        templateJson= std::string(argv[3]);
        outputPath= std::string(argv[4]);
    } else {
        std::cout << "Usage: OptimizeTrajectory (optimization config) (urdf) (contraints config)" << std::endl;
        return -3;
    }
    if(argc > 5) { shiftscale= std::atof(argv[5]); }

    Eigen::Vector3d minLimit(-2, -2, 0.2);

    urdf= ARDL::Util::getModelFromGazeboPath(urdf);
    std::shared_ptr<Chain<double>> c;
    std::shared_ptr<ForwardKinematics<double>> fk;
    c= std::shared_ptr<Chain<double>>(new Chain<double>(urdf));
    fk= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(c));
    aligned_vector<AdjointSE3<double>> adjoints;
    adjoints.resize(c->getNumOfJoints() + 1);
    Eigen::Matrix<double, -1, 3> posEE;
    posEE.resize(samples, 3);
    Eigen::VectorXd q(c->getNumOfJoints());
    Eigen::VectorXd nullspaceTarg(c->getNumOfJoints());
    std::vector<std::pair<double, double>> limits;
    limits.resize(c->getNumOfJoints());
    c->getJointLimits(limits);
    for(size_t i= 0; i < c->getNumOfJoints(); i++) { nullspaceTarg(i)= ((limits[i].second + limits[i].first) / 2.0)+0.1; }

    aligned_vector<Jacobian<double>> jacs;
    ARDL::Util::init(jacs, c->getNumOfJoints());

    Eigen::Matrix<double, 3, -1> jacPos;
    jacPos.resize(3, c->getNumOfJoints());
    Eigen::Matrix<double, -1, 3> jacPinv;
    jacPinv.resize(c->getNumOfJoints(), 3);
    Eigen::Matrix<double, -1, -1> nullspace;
    nullspace.resize(c->getNumOfJoints(), c->getNumOfJoints());
    nullspace.setZero();

    for(size_t i= 0; i < samples; ++i) {
        c->random();
        c->updateMatricesOptim();
        fk->getAdjointsOptim(adjoints);
        posEE.row(i)= adjoints.back().getP();
    }
    Eigen::Vector3d max(posEE.col(0).maxCoeff(), posEE.col(1).maxCoeff(), posEE.col(2).maxCoeff());
    Eigen::Vector3d min(posEE.col(0).minCoeff(), posEE.col(1).minCoeff(), posEE.col(2).minCoeff());
    std::cout << "MAX " << max.transpose() << std::endl;
    std::cout << "MIN " << min.transpose() << std::endl;
    for(size_t i= 0; i < 3; i++) {
        if(min(i) < minLimit(i)) { min(i)= minLimit(i); }
    }
    std::cout << "MIN " << min.transpose() << std::endl;
    aligned_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> cubes;
    cubes.push_back({min, max});
    aligned_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> subcubes;
    subcubes= separateCube(min, max);
    cubes.insert(cubes.end(), subcubes.begin(), subcubes.end());
    for(std::pair<Eigen::Vector3d, Eigen::Vector3d> &sc: subcubes) {
        aligned_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> tmp= separateCube(sc.first, sc.second);

        cubes.insert(cubes.end(), tmp.begin(), tmp.end());
    }
    Pseudoinverse<double> pinvCalc(3, c->getNumOfJoints());
    for(size_t i= 0; i < cubes.size(); ++i) {
        rapidjson::Document MFTjson;
        std::ifstream ifs(templateJson);
        IStreamWrapper isw(ifs);
        MFTjson.ParseStream(isw);
        size_t iterations= 0;
        c->randomPos();
        q= c->getQ() * 0.2;
        q=nullspaceTarg;
        std::cout << "Initial Q: " << q.transpose() << std::endl;
        // q =q+0.05;
        Eigen::VectorXd q_delta(c->getNumOfJoints()), nullError(c->getNumOfJoints());
        Eigen::Vector3d center= (cubes[i].second + cubes[i].first) / 2.0;
        for(size_t i =0;i<3;i++){
        center(i) +=  (0.2*((cubes[i].second(i) - cubes[i].first(i)) / 2.0))*(cubes[i].second(i)>=0);
        center(i) -=  (-0.2*((cubes[i].second(i) - cubes[i].first(i)) / 2.0))*(cubes[i].second(i)<0);
        }
        Eigen::Vector3d error, errorLine;
        errorLine << 500.0, 500.0, 500.0;
        double errorNorm= 100;
        bool valid= true;
        do {
            valid= true;
            c->updateChainPos(q);
            c->updateMatricesOptim();
            fk->getBodyAdjointsOptim(adjoints,true);
            fk->getJacobians<ARDL::Frame::BODY>(adjoints, jacs);
            jacPos= jacs.back().block(0, 0, 3, c->getNumOfJoints());
            error= adjoints[0].getR()*center+adjoints[0].getP();
            errorNorm=error.norm();
            jacPinv= pinvCalc.compute(jacPos, 1e-10);
            q_delta= jacPinv  *1*error;

            nullspace= Eigen::MatrixXd::Identity(c->getNumOfJoints(), c->getNumOfJoints()) - (jacPinv * jacPos);
            nullError= (nullspaceTarg - q);
            q_delta+= (nullspace  * 1*(nullError));
            q+= q_delta * 0.001;
            iterations++;
            for(size_t t_i0= 0; t_i0 < c->getNumOfJoints(); t_i0++) {
                valid= valid && (limits[t_i0].first <= q(t_i0) && limits[t_i0].second >= q(t_i0));
            }
            valid = valid && !fk->collisionOptim();
        } while((errorNorm > 0.01 || !valid) && (iterations < 500000));
        double scale= 1;
        if(MFTjson.IsObject()) {
            if(MFTjson.HasMember("joint_limit_constraint")) {
                scale= MFTjson["joint_limit_constraint"]["scale"].GetDouble();
            }
        }
        // std::vector<std::pair<double, double>> limits;
        // limits.resize(c->getNumOfJoints());
        // c->getJointLimits(limits);
        // for(size_t t_i0= 0; t_i0 < c->getNumOfJoints(); t_i0++) {
        //     double shift= (limits[t_i0].second - limits[t_i0].first) * shiftscale * scale;
        //     if(limits[t_i0].first * scale > q(t_i0)) {
        //         q(t_i0)= limits[t_i0].first * scale + shift;
        //     } else if(limits[t_i0].second * scale < q(t_i0)) {
        //         q(t_i0)= limits[t_i0].second * scale - shift;
        //     }
        // }
        c->updateChainPos(q);
        c->updateMatricesOptim();
        fk->getAdjointsOptim(adjoints,true);
        valid= (cubes[i].first.array() < adjoints.back().getP().array() &&
                cubes[i].second.array() > adjoints.back().getP().array())
                   .all();
        std::cout << "VALID " << valid << std::endl;
        for(size_t t_i0= 0; t_i0 < c->getNumOfJoints(); t_i0++) {
            valid= valid && (limits[t_i0].first <= q(t_i0) && limits[t_i0].second >= q(t_i0));
        }
        std::cout << "VALID " << valid << std::endl;
        valid = valid&& !fk->collisionOptim();
        std::pair<Eigen::VectorXd, Eigen::VectorXd> limitsVec;
        c->getJointLimits(limitsVec);
        // error= center - adjoints.back().getP();

        std::cout << "VALID " << valid << std::endl;
        std::cout << "TARGET " << center.transpose() << std::endl;
        std::cout << "ERROR " << error.norm() << std::endl;
        std::cout << "ITERS " << iterations << std::endl;
        std::cout << "q " << q.transpose() << std::endl;
        std::cout << "limitsMin " << limitsVec.first.transpose() << std::endl;
        std::cout << "limitsMax " << limitsVec.second.transpose() << std::endl;
        std::cout << "NULLSPACE TARG" << nullspaceTarg.transpose() << std::endl;
        std::cout << "POSITION " << adjoints.back().getP().transpose() << std::endl;
        std::cout << "FINISHED " << i << std::endl;

        if(MFTjson.IsObject() && valid) {
            std::stringstream ss;
            ss << outputPath << "/Contraints_" << i << ".json";
            std::ofstream ofs(ss.str());
            rapidjson::OStreamWrapper osw(ofs);
            rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
            Value &limits= MFTjson["link_limit_constraints"];
            for(size_t j= 0; j < 3; ++j) {
                limits[limits.Size() - 1][j].SetDouble(cubes[i].second[j]);
                limits[limits.Size() - 1][j + 3].SetDouble(cubes[i].first[j]);
            }
            Value startQ(kArrayType);
            for(size_t i2= 0; i2 < c->getNumOfJoints(); i2++) { startQ.PushBack(q[i2], MFTjson.GetAllocator()); }
            MFTjson.AddMember("startQ", startQ, MFTjson.GetAllocator());

            writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
            MFTjson.Accept(writer);
            ofs.flush();
            ofs.close();
        }
    }
}

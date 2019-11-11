#pragma once
#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include "VisualizerBase.hpp"
#include <rapidjson/document.h>
#include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <chrono>

class MFTVisualizer : public VisualizerBase {
private:
    std::shared_ptr<ModifiedFourierTrajectory<double> > MFT;
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> elapsedTime;
    Eigen::VectorXd q;
public:
    MFTVisualizer(std::string trajectoryFile, std::string robotUrdf, size_t screenWidth = 800, size_t screenHeight = 450, std::string title = "MFTVisualizer") : VisualizerBase(robotUrdf, screenWidth, screenHeight, title) {
        // std::cout << "D1 " << (*configRobot)["trajectory_dof"].GetUint() << " " << robot->getNumOfJoints() << std::endl;
        MFT = std::shared_ptr<ModifiedFourierTrajectory<double> >(new ModifiedFourierTrajectory<double>());
        MFT->loadFromJSON(trajectoryFile);
        start_time = std::chrono::system_clock::now();
        q.resize(robot->getNumOfJoints());
    }

    void draw() override {
        end_time = std::chrono::system_clock::now();
        elapsedTime = end_time - start_time;
        MFT->update(elapsedTime.count());
        q = MFT->getPosition();
        robot->updateChain(q, Eigen::VectorXd::Zero(robot->getNumOfJoints()));
        robot->updateMatrices();
        VisualizerBase::draw();
    }
};

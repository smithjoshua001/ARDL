#pragma once
#include <motion-generators/MultiModifiedFourierTrajectory.hpp>
#include "VisualizerBase.hpp"
#include <rapidjson/document.h>
// #include <motion-generators/ModifiedFourierTrajectory.hpp>
#include <chrono>
#include <GL/gl.h>

class MFTVisualizer : public VisualizerBase {
private:
    std::shared_ptr<JointTrajectory<double> > MFT;
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> elapsedTime;
    Eigen::VectorXd q;
    size_t samples;

    aligned_vector<AdjointSE3<double> > end_effector_poses;
    raylib::Vector3 p0, p1;

public:
    MFTVisualizer(std::string trajectoryFile, std::string robotUrdf, size_t screenWidth = 800, size_t screenHeight = 450, std::string title = "MFTVisualizer") : VisualizerBase(robotUrdf, screenWidth, screenHeight, title) {
        // std::cout << "D1 " << (*configRobot)["trajectory_dof"].GetUint() << " " << robot->getNumOfJoints() << std::endl;
        MFT = std::shared_ptr<ModifiedFourierTrajectory<double> >(new ModifiedFourierTrajectory<double>());
        MFT->loadFromJSON(trajectoryFile);
        start_time = std::chrono::system_clock::now();
        q.resize(robot->getNumOfJoints());
        samples = MFT->getPeriodLength() * 20;
        end_effector_poses.resize(samples);
        AdjointSE3<double> ee;
        for (size_t i = 0; i < samples; i++) {
            MFT->update(i / 20.0);
            robot->updateChain(MFT->getPosition(), MFT->getVelocity());
            robot->updateMatricesOptim();
            kin->getAdjointsOptim(ATs, false);
            kin->getEEAdjoint(ATs, ee);
            end_effector_poses[i] = ee;
        }
    }
    MFTVisualizer(std::vector<std::string> &trajectoryFile, std::string robotUrdf, size_t screenWidth = 800, size_t screenHeight = 450, std::string title = "MFTVisualizer") : VisualizerBase(robotUrdf, screenWidth, screenHeight, title) {
        MFT = std::shared_ptr<MultiModifiedFourierTrajectory<double> >(new MultiModifiedFourierTrajectory<double>(trajectoryFile, robot->getNumOfJoints()));
        // MFT->loadFromJSON(trajectoryFile);
        q.resize(robot->getNumOfJoints());
        samples = MFT->getPeriodLength() * 20;

        std::cout << "SAMPLES: " << samples << std::endl;
        end_effector_poses.resize(samples);
        AdjointSE3<double> ee;
        for (size_t i = 0; i < samples; i++) {
            MFT->update(i / 20.0);
            robot->updateChain(MFT->getPosition(), MFT->getVelocity());
            robot->updateMatricesOptim();
            kin->getAdjointsOptim(ATs, false);
            kin->getEEAdjoint(ATs, ee);
            end_effector_poses[i] = ee;
        }

        start_time = std::chrono::system_clock::now();
    }

    void draw() override {
        end_time = std::chrono::system_clock::now();
        elapsedTime = end_time - start_time;
        MFT->update(elapsedTime.count());
        q = MFT->getPosition();
            std::cout<<MFT->getPosition().transpose()<<std::endl;
        robot->updateChain(q, Eigen::VectorXd::Zero(robot->getNumOfJoints()));
        VisualizerBase::draw();
        glLineWidth(3.0f);
        rlBegin(RL_LINES);
        // glBegin(GL_LINES);
        raylib::Color r = RED;
        rlColor4ub(r.r, r.g, r.b, r.a);
        for (size_t i = 1; i < samples; i++) {
            p0.x = end_effector_poses[i - 1].getP()(0);
            p0.y = end_effector_poses[i - 1].getP()(1);
            p0.z = end_effector_poses[i - 1].getP()(2);
            p1.x = end_effector_poses[i].getP()(0);
            p1.y = end_effector_poses[i].getP()(1);
            p1.z = end_effector_poses[i].getP()(2);

            Vector3 tmp = Vector3Transform(p0, toggleYZ);
            Vector3 tmp2 = Vector3Transform(p1, toggleYZ);
            rlVertex3f(tmp.x, tmp.y, tmp.z);
            rlVertex3f(tmp2.x, tmp2.y, tmp2.z);

            // DrawLine3D(Vector3Transform(p0, toggleYZ), Vector3Transform(p1, toggleYZ), RED);
        }

        // glLineWidth(prev_width);
        rlEnd();
        // glLineWidth(1.0f);
        for (size_t i = 1; i < samples; i++) {
            p0.x = end_effector_poses[i - 1].getP()(0);
            p0.y = end_effector_poses[i - 1].getP()(1);
            p0.z = end_effector_poses[i - 1].getP()(2);
            p1.x = end_effector_poses[i].getP()(0);
            p1.y = end_effector_poses[i].getP()(1);
            p1.z = end_effector_poses[i].getP()(2);

            Vector3 tmp = Vector3Transform(p0, toggleYZ);
            Vector3 tmp2 = Vector3Transform(p1, toggleYZ);
            if (i % (samples / 60) == 0) {
                rlPushMatrix();
                Eigen::AngleAxisf coneRotate(Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitY(), toggleYZEigen.block<3, 3>(0, 0) * (end_effector_poses[i].getP() - end_effector_poses[i - 1].getP()).cast<float>()));
                // rlPushMatrix();
                rlTranslatef(tmp.x, tmp.y, tmp.z);
                rlRotatef(coneRotate.angle() * 180.0 / M_PI, coneRotate.axis()[0], coneRotate.axis()[1], coneRotate.axis()[2]);
                // rlRotatef(90, 1, 0, 0);
                DrawCylinder((Vector3){0, 0, 0}, 0, 0.015, 0.04, 8, BLUE);

                // rlTranslatef(camera.position.x, camera.position.y, camera.position.z);
                rlPopMatrix();
            }
        }
    }

    void postdraw() override {
        EndMode3D();

        DrawText("Free camera default controls:", 20, 20, 10, RAYWHITE);
        DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
        DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
        DrawText("- Alt + Mouse Wheel Pressed to Rotate", 40, 80, 10, DARKGRAY);
        DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", 40, 100, 10, DARKGRAY);
        DrawText("- Z to zoom to (0, 0, 0)", 40, 120, 10, DARKGRAY);

        if (std::dynamic_pointer_cast<MultiModifiedFourierTrajectory<double> >(MFT)) {
            DrawText(std::dynamic_pointer_cast<MultiModifiedFourierTrajectory<double> >(MFT)->getInfo().c_str(), 40, 140, 10, DARKGRAY);
        }

        DrawFPS(10, 10);
        EndDrawing();
    }
};

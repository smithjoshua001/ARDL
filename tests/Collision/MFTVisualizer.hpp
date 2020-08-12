#pragma once
#include "Collision/VisualizerBase.hpp"
#include <rapidjson/document.h>

#include <chrono>
#include <GL/gl.h>

class RobotVis : public VisualizerBase {
private:
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> elapsedTime;
    Eigen::VectorXd q,qd;

    raylib::Vector3 p0, p1;
    aligned_vector<AdjointSE3<double>> adjoints;

public:
    RobotVis(std::string robotUrdf, size_t screenWidth = 800, size_t screenHeight = 450, std::string title = "RobotVis") : VisualizerBase(robotUrdf, screenWidth, screenHeight, title) {
       
        q.resize(robot->getNumOfJoints());
q.setZero();
qd.resize(robot->getNumOfJoints());
qd.setZero();
        start_time = std::chrono::system_clock::now();

    adjoints.resize(robot->getNumOfJoints() + 1);
    std::cout<<robot->getNumOfJoints()<<std::endl;
    }

    void setQ(Eigen::VectorXd &q){
        this->q = q;

    robot->updateChain(q, qd);
    // robot->updateMatricesOptim();
    // kin->getBodyAdjointsOptim(adjoints,true);
    //         robot->updateChain(q, qd);
    robot->updateMatricesOptim();
    kin->getBodyAdjointsOptim(adjoints,true);
    // kin->getAdjointsOptim(adjoints,true);
    // robot->updateMatrices();
    // kin->getBodyAdjoints(adjoints,true);
    }

    void draw() override {
        end_time = std::chrono::system_clock::now();
        elapsedTime = end_time - start_time;

    robot->updateChain(q, qd);
    //             robot->updateMatrices();
    // kin->getBodyAdjoints(adjoints,true);

    robot->updateMatricesOptim();
    kin->getBodyAdjointsOptim(adjoints,true);
    // kin->getAdjointsOptim(adjoints,true);
        VisualizerBase::draw();

        if (IsKeyDown('1')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(0)-=0.01;
            }else{
q(0)+=0.01;
            }
        }
        if (IsKeyDown('2')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(1)-=0.01;
            }else{
q(1)+=0.01;
            }
        }
        if (IsKeyDown('3')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(2)-=0.01;
            }else{
q(2)+=0.01;
            }
        }
        if (IsKeyDown('4')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(3)-=0.01;
            }else{
q(3)+=0.01;
            }
        }
        if (IsKeyDown('5')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(4)-=0.01;
            }else{
q(4)+=0.01;
            }
        }
        if (IsKeyDown('6')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(5)-=0.01;
            }else{
q(5)+=0.01;
            }
        }
        if (IsKeyDown('7')){
            if(IsKeyDown(KEY_LEFT_SHIFT)){
            q(6)-=0.01;
            }else{
q(6)+=0.01;
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

        DrawFPS(10, 10);
        EndDrawing();
    }
};

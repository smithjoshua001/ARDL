#pragma once

#include "ARDL/Model/Chain.hpp"
#include "ARDL/Kinematics/ForwardKinematics.hpp"
namespace raylib {
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#define RLIGHTS_IMPLEMENTATION
#include "Collision/Shaders/rlights.h"
} // namespace raylib
#include <assimp/Importer.hpp>

// #if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION 330
#define GRAPHICS_API_OPENGL_33
// #else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
//     #define GLSL_VERSION            100
// #endif

using namespace ARDL::Model;
using namespace ARDL;
using namespace ARDL::Math;
using namespace raylib;
class VisualizerBase {
     protected:
    std::shared_ptr<Chain<double>> robot;
    std::shared_ptr<ForwardKinematics<double>> kin;
    size_t screenWidth, screenHeight;
    Camera3D camera;
    std::vector<raylib::Model> RobotLinks;
    raylib::Matrix toggleYZ;
    Eigen::Matrix4f toggleYZEigen;
    Shader shader;
    Light lights[MAX_LIGHTS];

    aligned_vector<Pose<double>> ATs;

     public:
    VisualizerBase(std::string robotUrdf, size_t screenWidth= 800, size_t screenHeight= 450,
                   std::string title= "Visualizer") {
        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        InitWindow(screenWidth, screenHeight, title.c_str());
        SetTargetFPS(60);
        // Define the camera to look into our 3d world
        camera= {0};
        camera.position= (Vector3){3.0f, 3.0f, 0.0f}; // Camera position
        camera.target= (Vector3){0.0f, 0.0f, 0.0f};   // Camera looking at point
        camera.up= (Vector3){0.0f, 1.0f, 0.0f};       // Camera up vector (rotation towards target)
        camera.fovy= 45.0f;                           // Camera field-of-view Y
        camera.type= CAMERA_PERSPECTIVE;              // Camera mode type
        SetCameraMode(camera, CAMERA_FREE);
        SetCameraPanControl(MOUSE_RIGHT_BUTTON);
        SetCameraAltControl(KEY_LEFT_SHIFT);

        robot= std::shared_ptr<Chain<double>>(new Chain<double>(robotUrdf));
        kin= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(robot));
        ATs.resize(robot->getNumOfJoints() + 1);

        toggleYZ= {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        toggleYZEigen << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            1.0f;


        for(size_t k= 0; k < robot->getMoveableLinksRef().size(); k++) {
            std::vector<Eigen::Vector3d> normals;

            Eigen::Matrix<double, Eigen::Dynamic, 3> verticies=
                robot->getMoveableLinksRef()[k]->getCollisionVerticesOptim();
            std::vector<std::array<int, 3>> tris= robot->getMoveableLinksRef()[k]->getCollisionTrianglesOptim();
            normals.resize(verticies.rows());
            for(size_t i= 0; i < normals.size(); i++) { normals[i].setZero(); }
            for(size_t i= 0; i < tris.size(); i++) {
                std::array<int, 3> &tri= tris[i];
                Eigen::Vector3d BA, CA, cross;
                BA= verticies.row(tri[1]) - verticies.row(tri[0]);
                CA= verticies.row(tri[2]) - verticies.row(tri[0]);
                cross= BA.cross(CA);
                normals[tri[0]]+= cross;

                normals[tri[1]]+= cross;

                normals[tri[2]]+= cross;
            }
            for(size_t i= 0; i < normals.size(); i++) { normals[i].normalize(); }

            // RobotLinks.push_back({0});
            Mesh link;
            memset(&link, 0, sizeof(Mesh));
            link.vertexCount= verticies.rows();
            link.triangleCount= tris.size();

            link.vertices= (float *) RL_CALLOC(link.vertexCount * 3, sizeof(float));
            link.texcoords= (float *) RL_CALLOC(link.vertexCount * 2, sizeof(float));
            link.normals= (float *) RL_CALLOC(link.vertexCount * 3, sizeof(float));
            link.indices= (unsigned short *) RL_MALLOC(link.triangleCount * 3 * sizeof(unsigned short));

            for(size_t i= 0; i < link.vertexCount; i++) {
                link.vertices[i * 3]= verticies(i, 0);
                link.vertices[i * 3 + 1]= verticies(i, 1);
                link.vertices[i * 3 + 2]= verticies(i, 2);
                link.normals[i * 3]= normals[i][0];
                link.normals[i * 3 + 1]= normals[i][1];
                link.normals[i * 3 + 2]= normals[i][2];
                link.texcoords[i * 2 + 0]= 0; // normals[i][0];
                link.texcoords[i * 2 + 1]= 0; // normals[i][1];
            }
            for(size_t i= 0; i < link.triangleCount; i++) {
                link.indices[i * 3]= tris[i][0];
                link.indices[i * 3 + 1]= tris[i][1];
                link.indices[i * 3 + 2]= tris[i][2];
            }
            RobotLinks.push_back(LoadModelFromMesh(link));
            RobotLinks.back().transform= MatrixIdentity();
            rlLoadMesh(&RobotLinks.back().meshes[0], false);

        }


        Eigen::VectorXd tmp= Eigen::VectorXd::Zero(robot->getNumOfJoints());
        robot->updateChain(tmp, Eigen::VectorXd::Zero(robot->getNumOfJoints()));
        robot->updateMatricesOptim();

        kin->getBodyAdjointsOptim(ATs, true);

        shader= LoadShader(
            "/home/joshua/ssd/Projects/Projects/PhD/ARDL_ws/src/ARDL/tests/Collision/Shaders/basic_lighting.vs",
            "/home/joshua/ssd/Projects/Projects/PhD/ARDL_ws/src/ARDL/tests/Collision/Shaders/basic_lighting.fs");
        shader.locs[LOC_MATRIX_MODEL]= GetShaderLocation(shader, "matModel");
        shader.locs[LOC_VECTOR_VIEW]= GetShaderLocation(shader, "viewPos");
        int ambientLoc= GetShaderLocation(shader, "ambient");
        float ambient[4]= {0.2f, 0.2f, 0.2f, 1.0f};
        SetShaderValue(shader, ambientLoc, ambient, UNIFORM_VEC4);

        for(size_t i= 0; i < RobotLinks.size(); i++) { RobotLinks[i].materials[0].shader= shader; }
        lights[0]= CreateLight(LIGHT_POINT, (Vector3){-5, 2, 1}, Vector3Zero(), WHITE, shader);
        lights[0].enabled= true;
        lights[1]= CreateLight(LIGHT_POINT, (Vector3){5, 2, 1}, Vector3Zero(), WHITE, shader);
        lights[1].enabled= true;
    }

    ~VisualizerBase() {
        if(IsWindowReady()) CloseWindow();
    }

    virtual void predraw() {
        UpdateCamera(&camera);
        if(IsKeyDown('Z')) camera.target= (Vector3){0.0f, 0.0f, 0.0f};
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
    }

    virtual void postdraw() {
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

    virtual void draw() {
        // kin->getAdjoints(ATs, true);
        const std::vector<std::pair<size_t, size_t>> &collisionPairs= kin->getCollisionPairsOptim();

        float cameraPos[3]= {camera.position.x, camera.position.y, camera.position.z};
        SetShaderValue(shader, shader.locs[LOC_VECTOR_VIEW], cameraPos, UNIFORM_VEC3);
        for(size_t i= 0; i < RobotLinks.size(); i++) { // RobotLinks.size()
            Eigen::Vector3d p= robot->getMoveableLinksRef()[i]->getCollisionObjectOptimPtr()->getTranslation();
            Eigen::Matrix3d R= robot->getMoveableLinksRef()[i]->getCollisionObjectOptimPtr()->getRotation();
            raylib::Matrix LinkTransform= {(float) R(0, 0),
                                           (float) R(0, 1),
                                           (float) R(0, 2),
                                           (float) p(0),
                                           (float) R(1, 0),
                                           (float) R(1, 1),
                                           (float) R(1, 2),
                                           (float) p(1),
                                           (float) R(2, 0),
                                           (float) R(2, 1),
                                           (float) R(2, 2),
                                           (float) p(2),
                                           0,
                                           0,
                                           0,
                                           1};
            RobotLinks[i].transform= MatrixMultiply(LinkTransform, toggleYZ);
            bool drawn= false;
            for(size_t i2= 0; i2 < collisionPairs.size(); i2++) {
                if(i == collisionPairs[i2].first || i == collisionPairs[i2].second) {
                    DrawModelWires(RobotLinks[i], {0.0f, 0.0f, 0.0f}, 1.0f, RED);
                    drawn= true;
                    break;
                }
            }
            if(!drawn) { DrawModel(RobotLinks[i], {0.0f, 0.0f, 0.0f}, 1.0f, GRAY); }
        }
        DrawGrid(10, 1.0f);
        Vector3 x= {1, 0, 0};
        Vector3 y= {0, 1, 0};
        Vector3 z= {0, 0, 1};
        x= {2, 0, 0};
        y= {0, 2, 0};
        z= {0, 0, 2};
        x= Vector3Transform(x, toggleYZ);
        y= Vector3Transform(y, toggleYZ);
        z= Vector3Transform(z, toggleYZ);
        DrawSphereEx(x, 0.05f, 8, 8, WHITE);
        DrawSphereEx(y, 0.05f, 8, 8, BLUE);
        DrawSphereEx(z, 0.05f, 8, 8, GREEN);
    }

    void run() {
        while(!WindowShouldClose()) {
            predraw();
            draw();
            postdraw();
        }
    }

    void close() { CloseWindow(); }
};

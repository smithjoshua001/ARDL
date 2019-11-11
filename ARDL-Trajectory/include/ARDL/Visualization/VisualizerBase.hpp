#pragma once

#include "ARDL/Model/Chain.hpp"
#include "ARDL/Kinematics/ForwardKinematics.hpp"
namespace raylib {
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#define RLIGHTS_IMPLEMENTATION
#include "ARDL/Visualization/Shaders/rlights.h"
}
#include <assimp/Importer.hpp>

// #if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
// #else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
//     #define GLSL_VERSION            100
// #endif

using namespace ARDL::Model;
using namespace ARDL;
using namespace ARDL::Math;
using namespace raylib;
class VisualizerBase {
protected:
    std::shared_ptr<Chain<double> > robot;
    std::shared_ptr<ForwardKinematics<double> > kin;
    size_t screenWidth, screenHeight;
    Camera3D camera;
    std::vector<raylib::Model> RobotLinks;
    Matrix toggleYZ;
    Shader shader;
    Light lights[MAX_LIGHTS];

    aligned_vector<AdjointSE3<double> > ATs;
public:
    VisualizerBase(std::string robotUrdf, size_t screenWidth = 800, size_t screenHeight = 450, std::string title = "Visualizer") {
        std::cout << "D1" << std::endl;
        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        InitWindow(screenWidth, screenHeight, title.c_str());
        SetTargetFPS(60);
        // Define the camera to look into our 3d world
        camera = {0};
        camera.position = (Vector3){
            3.0f, 3.0f, 0.0f
        };  // Camera position
        camera.target = (Vector3){
            0.0f, 0.0f, 0.0f
        };      // Camera looking at point
        camera.up = (Vector3){
            0.0f, 1.0f, 0.0f
        };          // Camera up vector (rotation towards target)
        camera.fovy = 45.0f;                                // Camera field-of-view Y
        camera.type = CAMERA_PERSPECTIVE;                   // Camera mode type
        SetCameraMode(camera, CAMERA_FREE);
        SetCameraPanControl(MOUSE_RIGHT_BUTTON);
        SetCameraAltControl(KEY_LEFT_SHIFT);

        std::cout << "D2" << std::endl;
        robot = std::shared_ptr<Chain<double> >(new Chain<double>(robotUrdf));
        kin = std::shared_ptr<ForwardKinematics<double> >(new ForwardKinematics<double>(robot));
        ATs.resize(robot->getNumOfLinks() + 1);
        const aligned_vector<std::shared_ptr<Link<double> > > links = robot->getLinks();

        std::cout << "D3" << std::endl;
        toggleYZ = {1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, -1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f};

        std::cout << "D4" << std::endl;
        for (size_t i = 0; i < robot->getLinks().size(); i++) {
            // RobotLinks.push_back({0});
            // RobotLinks.back().transform = MatrixIdentity();
            // RobotLinks.back().meshCount = 1;
            // RobotLinks.back().meshes = (Mesh *)RL_CALLOC(RobotLinks.back().meshCount, sizeof(Mesh));

            // Eigen::MatrixXf verts = links[i]->getCollisionVectors().cast<float>();
            // link.vertices = verts.data();
            // link.vertexCount = verts.rows();
            // std::vector<std::array<int, 3> > indicies = links[i]->getCollisionTriangles();

            // std::vector<std::array<unsigned short, 3> > shortIndicies;
            // size_t j = 0;
            // shortIndicies.resize(indicies.size());
            // for (std::array<int, 3> &indexes: indicies) {
            //     shortIndicies[i][0] = indexes[0];
            //     shortIndicies[i][1] = indexes[1];
            //     shortIndicies[i][2] = indexes[2];
            // }
            // link.indices = shortIndicies.data()->data();

            // links[i]->getCollisionModel();
            std::vector<std::array<float, 3> > verts;
            std::vector<std::array<unsigned short, 3> > faces;
            std::vector<std::array<float, 3> > normals;
            std::vector<std::array<float, 4> > tangents;
            std::vector<std::array<unsigned char, 4> > colors;
            std::vector<std::array<float, 2> > tex;
            urdf::LinkConstSharedPtr urdfModel = links[i]->getUrdfLink();

            Assimp::Importer Importer;
            std::shared_ptr<urdf::Mesh> t_mesh_sp;
            std::shared_ptr<urdf::Cylinder> cylinder;
            std::shared_ptr<urdf::Box> box;
            std::shared_ptr<urdf::Sphere> sphere;

            std::cout << "D5" << std::endl;
            // if(urdfModel->visual_array.size()>1){
            for (size_t i1 = 0; i1 < urdfModel->visual_array.size(); i1++) {
                if ((t_mesh_sp = std::dynamic_pointer_cast<urdf::Mesh>(urdfModel->visual_array[i1]->geometry))) {
                    std::cout << "D6" << std::endl;
                    // const aiScene *assimp_model = Importer.ReadFile(t_mesh_sp->filename,
                    //                                                 aiProcess_Triangulate | aiProcess_OptimizeMeshes | aiProcess_RemoveRedundantMaterials |
                    //                                                 aiProcess_JoinIdenticalVertices | aiProcess_GenUVCoords | aiProcess_GenSmoothNormals | aiProcess_FixInfacingNormals | aiProcess_CalcTangentSpace |
                    //                                                 aiProcess_SortByPType);
                    std::string filename = ARDL::Util::extractModelPath(std::string(t_mesh_sp->filename));
                    const aiScene *assimp_model = Importer.ReadFile(filename,
                                                                    aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenUVCoords | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace | aiProcess_SortByPType);

                    std::cout << "D6.1" << std::endl;
                    if (assimp_model == NULL) {
                        std::cout << "NO ASSIMP MODEL?" << std::endl;
                        exit(-3);
                    }
                    // for (size_t i2 = 0; i2 < assimp_model->mNumMeshes; i2++) {
                    size_t i2 = 0;
                    std::cout << "D6.1.1" << std::endl;
                    for (size_t i3 = 0; i3 < assimp_model->mMeshes[i2]->mNumVertices; i3++) {
                        verts.push_back({(float)t_mesh_sp->scale.x * assimp_model->mMeshes[i2]->mVertices[i3][0], (float)t_mesh_sp->scale.y * assimp_model->mMeshes[i2]->mVertices[i3][1], (float)t_mesh_sp->scale.z * assimp_model->mMeshes[i2]->mVertices[i3][2]});
                        normals.push_back({assimp_model->mMeshes[i2]->mNormals[i3][0], assimp_model->mMeshes[i2]->mNormals[i3][1], assimp_model->mMeshes[i2]->mNormals[i3][2]});
                        if (assimp_model->mMeshes[i2]->mTangents != NULL) {
                            std::cout << assimp_model->mMeshes[i2]->mTangents[i3].x << std::endl;
                            Eigen::Quaternionf q;
                            q = Eigen::AngleAxisf(assimp_model->mMeshes[i2]->mTangents[i3][0], Eigen::Vector3f::UnitX())
                                * Eigen::AngleAxisf(assimp_model->mMeshes[i2]->mTangents[i3][1], Eigen::Vector3f::UnitY())
                                * Eigen::AngleAxisf(assimp_model->mMeshes[i2]->mTangents[i3][2], Eigen::Vector3f::UnitZ());

                            tangents.push_back({q.x(), q.y(), q.z(), q.w()});
                        }
                        if (assimp_model->mMeshes[i2]->mColors[0] != NULL) {
                            colors.push_back({(unsigned char)assimp_model->mMeshes[i2]->mColors[0]->r, (unsigned char)assimp_model->mMeshes[i2]->mColors[0]->g, (unsigned char)assimp_model->mMeshes[i2]->mColors[0]->b, (unsigned char)assimp_model->mMeshes[i2]->mColors[0]->a});
                        }

                        if (assimp_model->mMeshes[i2]->mTextureCoords[0] != NULL) {
                            tex.push_back({assimp_model->mMeshes[i2]->mTextureCoords[0]->x, assimp_model->mMeshes[i2]->mTextureCoords[0]->y});
                        }
                    }

                    for (size_t i3 = 0; i3 < assimp_model->mMeshes[i2]->mNumFaces; i3++) {
                        faces.push_back({(unsigned short)assimp_model->mMeshes[i2]->mFaces[i3].mIndices[0], (unsigned short)assimp_model->mMeshes[i2]->mFaces[i3].mIndices[1], (unsigned short)assimp_model->mMeshes[i2]->mFaces[i3].mIndices[2]});
                    }
                    // }

                    std::cout << "D7" << std::endl;
                    RobotLinks.push_back({0});
                    // aiString path;
                    // if (assimp_model->mMaterials[0]->GetTextureCount(aiTextureType::aiTextureType_DIFFUSE) > 0) {
                    //     assimp_model->mMaterials[0]->GetTexture(aiTextureType::aiTextureType_DIFFUSE, 0, &path);
                    // } else {
                    //     exit(-2);
                    // }
                    Mesh link;
                    memset(&link, 0, sizeof(Mesh));
                    link.vertexCount = verts.size();
                    link.triangleCount = faces.size();

                    std::cout << "D7.1 " << verts.size() << std::endl;
                    link.vertices = (float *)RL_CALLOC(link.vertexCount * 3, sizeof(float));
                    link.texcoords = (float *)RL_CALLOC(link.vertexCount * 2, sizeof(float));
                    link.normals = (float *)RL_CALLOC(link.vertexCount * 3, sizeof(float));
                    link.indices = (unsigned short *)RL_MALLOC(link.triangleCount * 3 * sizeof(unsigned short));

                    std::cout << "D7.2" << std::endl;
                    for (size_t i = 0; i < link.vertexCount; i++) {
                        link.vertices[i * 3] = verts[i][0];
                        link.vertices[i * 3 + 1] = verts[i][1];
                        link.vertices[i * 3 + 2] = verts[i][2];
                        link.normals[i * 3] = normals[i][0];
                        link.normals[i * 3 + 1] = normals[i][1];
                        link.normals[i * 3 + 2] = normals[i][2];
                        link.texcoords[i * 2 + 0] = 0;//normals[i][0];
                        link.texcoords[i * 2 + 1] = 0;//normals[i][1];
                    }
                    for (size_t i = 0; i < link.triangleCount; i++) {
                        link.indices[i * 3] = faces[i][0];
                        link.indices[i * 3 + 1] = faces[i][1];
                        link.indices[i * 3 + 2] = faces[i][2];
                    }

                    RobotLinks.back().meshCount = 1;
                    // std::cout << "D7.4" << std::endl;
                    RobotLinks.back().meshes = (Mesh *)RL_CALLOC(RobotLinks.back().meshCount, sizeof(Mesh));
                    // std::cout << "D8" << std::endl;
                    RobotLinks.back().meshes[0] = link;
                    RobotLinks.back().materialCount = 1;
                    RobotLinks.back().materials = (Material *)RL_CALLOC(RobotLinks.back().materialCount, sizeof(Material));
                    RobotLinks.back().materials[0] = LoadMaterialDefault();

                    RobotLinks.back().meshMaterial = (int *)RL_CALLOC(RobotLinks.back().meshCount, sizeof(int));
                    RobotLinks.back().meshMaterial[0] = 0;  // First material index
                    std::cout << "D8" << std::endl;
                    Eigen::Quaterniond quat;
                    urdfModel->visual_array[i1]->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                    double r, p, y;
                    urdfModel->visual_array[i1]->origin.rotation.getRPY(r, p, y);
                    std::cout << "r: " << r << " y: " << y << " p: " << p << std::endl;
                    std::cout << "D8.1" << std::endl;
                    Eigen::AngleAxisf aa(quat.cast<float>());

                    Vector3 rotation = {aa.axis()[0], aa.axis()[1], aa.axis()[2]};
                    std::cout << "D8.2: " << aa.axis().transpose() << std::endl;
                    Matrix translate = MatrixTranslate(urdfModel->visual_array[i1]->origin.position.x, urdfModel->visual_array[i1]->origin.position.y, urdfModel->visual_array[i1]->origin.position.z);
                    Matrix rotating = MatrixMultiply(MatrixMultiply(MatrixRotateX(r), MatrixRotateY(p)), MatrixRotateZ(y));//MatrixRotate(rotation, aa.angle());
                    // Matrix lower = MatrixMultiply(toggleYZ, MatrixMultiply(MatrixRotateZ(-M_PI / 2), MatrixMultiply(MatrixRotateX(-M_PI / 2), MatrixMultiply(rotating, translate))));
                    Matrix lower = MatrixMultiply(MatrixMultiply(MatrixMultiply(MatrixMultiply(rotating, translate), MatrixRotateX(0)), MatrixRotateZ(0)), MatrixIdentity());
                    std::cout << "D8.3" << std::endl;
                    for (size_t i2 = 0; i2 < RobotLinks.back().meshes[0].vertexCount; i2++) {
                        Vector3 vertexTmp = {RobotLinks.back().meshes[0].vertices[0 + i2 * 3], RobotLinks.back().meshes[0].vertices[1 + i2 * 3], RobotLinks.back().meshes[0].vertices[2 + i2 * 3]};
                        Vector3 transVertex = Vector3Transform(vertexTmp, lower);
                        RobotLinks.back().meshes[0].vertices[0 + i2 * 3] = transVertex.x;
                        RobotLinks.back().meshes[0].vertices[1 + i2 * 3] = transVertex.y;
                        RobotLinks.back().meshes[0].vertices[2 + i2 * 3] = transVertex.z;
                    }
                    std::cout << "D8.4" << std::endl;
                    rlLoadMesh(&RobotLinks.back().meshes[0], false);
                    // std::cout << "D7.3" << std::endl;
                    // std::cout << "D8.0.1" << std::endl;
                    // RobotLinks.back().meshMaterial = (int *)RL_CALLOC(1, sizeof(int));
                    // RobotLinks.back().meshMaterial[0] = 0;
                    // std::cout << "D8.1" << std::endl;

                    // RobotLinks.back().materialCount = 1;
                    // RobotLinks.back().materials = (Material *)RL_CALLOC(RobotLinks.back().materialCount, sizeof(Material));

                    // RobotLinks.back().materials[0] = LoadMaterialDefault();
                    // RobotLinks.back().materials[0].maps[MAP_DIFFUSE].texture = GetTextureDefault();
                    // std::cout << "D8.2" << std::endl;
                    // RobotLinks.back().materials[0].maps[MAP_DIFFUSE].color = (Color){
                    //     (unsigned char)(0.5 * 255.0f), (unsigned char)(0.5 * 255.0f), (unsigned char)(0.5 * 255.0f), 255
                    // };
                    // std::cout << "D8.3" << std::endl;
                    // RobotLinks.back().materials[0].maps[MAP_DIFFUSE].value = 0.0f;
                    // std::cout << "D8.4" << std::endl;
                    // RobotLinks.back().materials[0].maps[MAP_SPECULAR].color = (Color){
                    //     (unsigned char)(0.5 * 255.0f), (unsigned char)(0.5 * 255.0f), (unsigned char)(0.5 * 255.0f), 255
                    // };
                    // std::cout << "D8.5" << std::endl;
                    // RobotLinks.back().materials[0].maps[MAP_SPECULAR].value = 0.0f;
                    // std::cout << "D8.6" << std::endl;
                    // RobotLinks.back().materials[0].maps[MAP_NORMAL].color = WHITE;
                    // RobotLinks.back().materials[0].maps[MAP_NORMAL].value = 0.5;
                    // std::cout << "D9" << std::endl;

                    // RobotLinks.back().materials[0].maps[MAP_EMISSION].color = (Color){
                    //     (unsigned char)(0 * 255.0f), (unsigned char)(0 * 255.0f), (unsigned char)(0 * 255.0f), 255
                    // };
                } else if ((cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(urdfModel->visual_array[i1]->geometry))) {
                    RobotLinks.push_back(LoadModelFromMesh(GenMeshCylinder(cylinder->radius, cylinder->length, 16)));
                    Eigen::Quaterniond quat;
                    urdfModel->visual_array[i1]->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                    Eigen::AngleAxisf aa(quat.cast<float>());

                    Vector3 rotation = {aa.axis()[0], aa.axis()[1], aa.axis()[2]};

                    Matrix lower = MatrixMultiply(toggleYZ, MatrixMultiply(MatrixTranslate(urdfModel->visual_array[i1]->origin.position.x, urdfModel->visual_array[i1]->origin.position.y, urdfModel->visual_array[i1]->origin.position.z + cylinder->length / 2.0), MatrixRotate(rotation, aa.angle())));
                    for (size_t i2 = 0; i2 < RobotLinks.back().meshes[0].vertexCount; i2++) {
                        Vector3 vertexTmp = {RobotLinks.back().meshes[0].vertices[0 + i2 * 3], RobotLinks.back().meshes[0].vertices[1 + i2 * 3], RobotLinks.back().meshes[0].vertices[2 + i2 * 3]};
                        Vector3 transVertex = Vector3Transform(vertexTmp, lower);
                        RobotLinks.back().meshes[0].vertices[0 + i2 * 3] = transVertex.x;
                        RobotLinks.back().meshes[0].vertices[1 + i2 * 3] = transVertex.y;
                        RobotLinks.back().meshes[0].vertices[2 + i2 * 3] = transVertex.z;
                        rlUpdateMesh(RobotLinks.back().meshes[0], 0, RobotLinks.back().meshes[0].vertexCount);
                    }
                    RobotLinks.back().transform = MatrixIdentity();
                }
            }
        }

        Eigen::VectorXd tmp = Eigen::VectorXd::Zero(robot->getNumOfJoints());
        robot->updateChain(tmp, Eigen::VectorXd::Zero(robot->getNumOfJoints()));
        robot->updateMatrices();

        kin->getLinkAdjoints(ATs, false);

        // shader = LoadShader("/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/include/ARDL/Visualization/Shaders/basic_lighting.vs", "/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/include/ARDL/Visualization/Shaders/basic_lighting.fs");
        // shader.locs[LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
        // shader.locs[LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
        // int ambientLoc = GetShaderLocation(shader, "ambient");
        // float ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
        // SetShaderValue(shader, ambientLoc, ambient, UNIFORM_VEC4);

        // for (size_t i = 0; i < RobotLinks.size(); i++) {
        //     RobotLinks[i].materials[0].shader = shader;
        // }
        // lights[0] = CreateLight(LIGHT_POINT, (Vector3){-5, 2, -1}, Vector3Zero(), WHITE, shader);
        // lights[0].enabled = true;
    }

    ~VisualizerBase() {
        if (IsWindowReady())
            CloseWindow();
    }

    virtual void draw() {
        // Eigen::VectorXd tmp = robot->getQ();
        // for (size_t i = 0; i < robot->getNumOfJoints(); i++) {
        //     tmp(i) += 0.001;
        // }
        // robot->updateChain(tmp, Eigen::VectorXd::Zero(robot->getNumOfJoints()));
        // robot->updateMatrices();
        kin->getLinkAdjoints(ATs, true);
        // if (kin->collision()) {
        //     std::cout << "Collide" << std::endl;
        // }
        const std::vector<std::pair<size_t, size_t> > &collisionPairs = kin->getCollisionPairs();
        UpdateCamera(&camera);
        if (IsKeyDown('Z'))
            camera.target = (Vector3){
                0.0f, 0.0f, 0.0f
            };
        // float cameraPos[3] = {camera.position.x, camera.position.y, camera.position.z};
        // SetShaderValue(shader, shader.locs[LOC_VECTOR_VIEW], cameraPos, UNIFORM_VEC3);
        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera);
        // RobotLinks.back().transform = MatrixMultiply(MatrixMultiply(toggleYZ, MatrixRotateX(M_PI / 2)), MatrixMultiply(MatrixTranslate(1, 0.5, 2), MatrixMultiply(MatrixRotateX(-M_PI / 2), toggleYZ)));
        // std::cout << "ATS SIZE: " << ATs.size() << std::endl;
        for (size_t i = 0; i < RobotLinks.size(); i++) {
            // size_t i = 2;
            // std::cout << "i: " << i << ": " << ATs[i].getP().transpose() << std::endl;
            // std::cout << ATs[i].getR() << std::endl;
            Matrix LinkTransform = {(float)ATs[i].getR()(0, 0), (float)ATs[i].getR()(0, 1), (float)ATs[i].getR()(0, 2), (float)ATs[i].getP()(0),
                                    (float)ATs[i].getR()(1, 0), (float)ATs[i].getR()(1, 1), (float)ATs[i].getR()(1, 2), (float)ATs[i].getP()(1),
                                    (float)ATs[i].getR()(2, 0), (float)ATs[i].getR()(2, 1), (float)ATs[i].getR()(2, 2), (float)ATs[i].getP()(2),
                                    0, 0, 0, 1};
            // RobotLinks[i].transform = MatrixMultiply(MatrixRotateX(0), MatrixMultiply(LinkTransform, MatrixRotateX(M_PI / 2)));
            // RobotLinks[i].transform = LinkTransform;
            RobotLinks[i].transform = MatrixMultiply(LinkTransform, toggleYZ);
            bool drawn = false;
            for (size_t i2 = 0; i2 < collisionPairs.size(); i2++) {
                if (i == collisionPairs[i2].first || i == collisionPairs[i2].second) {
                    DrawModelWires(RobotLinks[i], {0.0f, 0.0f, 0.0f}, 1.0f, RED);
                    drawn = true;
                    break;
                }
            }
            if (!drawn) {
                DrawModelWires(RobotLinks[i], {0.0f, 0.0f, 0.0f}, 1.0f, GREEN);
            }
        }
        DrawGrid(10, 1.0f);
        // if (lights[0].enabled) {
        //     DrawSphereEx(lights[0].position, 0.2f, 8, 8, WHITE);
        // }
        Vector3 x = {1, 0, 0};
        Vector3 y = {0, 1, 0};
        Vector3 z = {0, 0, 1};
        // DrawSphereEx(x, 0.05f, 8, 8, WHITE);
        // DrawSphereEx(y, 0.2f, 8, 8, BLUE);
        // DrawSphereEx(z, 0.2f, 8, 8, GREEN);
        x = {2, 0, 0};
        y = {0, 2, 0};
        z = {0, 0, 2};
        x = Vector3Transform(x, toggleYZ);
        y = Vector3Transform(y, toggleYZ);
        z = Vector3Transform(z, toggleYZ);
        DrawSphereEx(x, 0.05f, 8, 8, WHITE);
        DrawSphereEx(y, 0.05f, 8, 8, BLUE);
        DrawSphereEx(z, 0.05f, 8, 8, GREEN);

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

    void run() {
        while (!WindowShouldClose()) {
            draw();
        }
    }

    void close() {
        CloseWindow();
    }
};

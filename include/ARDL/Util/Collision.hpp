#pragma once

#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <fcl/fcl.h>

#include <fstream>

namespace ARDL {
    namespace Util {
        static std::string extractModelPath(std::string filename) {
            if (filename.find("model://") != std::string::npos) {
                std::string gz_model_paths = std::getenv("GAZEBO_MODEL_PATH");
                std::cout << "MODEL PATHS=" << gz_model_paths << std::endl;
                std::istringstream iss(gz_model_paths);
                std::string possible_path;
                while (std::getline(iss, possible_path, ':')) {
                    std::cout << "New PATH=" << (possible_path + filename.substr(7)) << std::endl;
                    if (std::ifstream((possible_path + filename.substr(7)).c_str())) {
                        filename = (possible_path + filename.substr(7));
                        break;
                    }
                }
            }
            return filename;
        }
        namespace Collision {
            /**
             * @brief Add a collision mesh object to FCL model
             *
             * @tparam BV
             * @param model input model
             * @param scale scale of mesh compared to real world
             * @param filename File containing mesh detail
             * @param offset Location of mesh in 3D space
             * @throw std::runtime_error if the mesh cannot be loaded somehow
             */
            template<class BV, typename T> static void createMesh(std::shared_ptr<::fcl::BVHModel<BV> > model, Eigen::Matrix<T, 3, 1> scale, std::string &filename, const fcl::Transform3<T> &offset) {
                aiPropertyStore *propertyStore = aiCreatePropertyStore();
                aiSetImportPropertyInteger(propertyStore,
                                           AI_CONFIG_PP_SBP_REMOVE,
                                           aiPrimitiveType_POINT
                                           | aiPrimitiveType_LINE);

                filename = extractModelPath(filename);
                std::ifstream t(filename);
                std::string input_model((std::istreambuf_iterator<char>(t)),
                                        std::istreambuf_iterator<char>());

                // std::cout << "input mesh file: " << input_model << std::endl;
                std::string extension;
                const std::size_t extensionIndex = filename.find_last_of('.');
                if (extensionIndex != std::string::npos)
                    extension = filename.substr(extensionIndex + 1);
                std::transform(std::begin(extension), std::end(extension),
                               std::begin(extension), ::tolower);

                const aiScene *_mesh = aiImportFileFromMemoryWithProperties(input_model.c_str(), input_model.size(), aiProcess_GenNormals
                                                                            | aiProcess_Triangulate
                                                                            | aiProcess_JoinIdenticalVertices
                                                                            | aiProcess_SortByPType
                                                                            | aiProcess_OptimizeMeshes, extension.c_str(), propertyStore);
                if (!_mesh) {
                    throw std::runtime_error("MODEL NOT LOADED!!!");
                }

                if (extension == "dae" || extension == "zae") {
                    _mesh->mRootNode->mTransformation = aiMatrix4x4();
                }

                // Finally, pre-transform the vertices. We can't do this as part of the
                // import process, because we may have changed mTransformation above.
                _mesh = aiApplyPostProcessing(_mesh, aiProcess_PreTransformVertices);
                if (!_mesh) {
                    throw std::runtime_error("MODEL NOT LOADED!!!");
                }

                assert(_mesh);
                if (model->build_state == fcl::BVH_BUILD_STATE_EMPTY) {
                    model->beginModel();
                }
                for (std::size_t i = 0; i < _mesh->mNumMeshes; i++) {
                    for (std::size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++) {
                        Eigen::Matrix<T, 3, 1> vertices[3];
                        for (std::size_t k = 0; k < 3; k++) {
                            const aiVector3D &vertex =
                                _mesh->mMeshes[i]->mVertices[
                                    _mesh->mMeshes[i]->mFaces[j].mIndices[k]];
                            vertices[k] = Eigen::Matrix<T, 3, 1>(vertex.x * scale[0],
                                                                 vertex.y * scale[1],
                                                                 vertex.z * scale[2]);
                            vertices[k] = offset * vertices[k];
                        }
                        model->addTriangle(vertices[0], vertices[1], vertices[2]);
                    }
                }
            }

            /**
             * @brief Add a collision cylinder object to FCL model
             *
             * @tparam BV
             * @param model
             * @param radius
             * @param length
             * @param offset
             */
            template<class BV, typename T> static void createCylinder(std::shared_ptr<::fcl::BVHModel<BV> > model, T radius, T length, const fcl::Transform3<T> &offset) {
                generateBVHModel(*model, fcl::Cylinder<T>(radius, length), offset, 128, fcl::FinalizeModel::DONT);
            }

            /**
             * @brief Add a collision box object to FCL model
             *
             * @tparam BV
             * @param model
             * @param size
             * @param offset
             */
            template<class BV, typename T> static void createBox(std::shared_ptr<::fcl::BVHModel<BV> > model, const Eigen::Matrix<T, 3, 1> &size, const fcl::Transform3<T> &offset) {
                generateBVHModel(*model, fcl::Box<T>(size), offset, fcl::FinalizeModel::DONT);
            }

            /**
             * @brief Add a collision sphere object to FCL model
             *
             * @tparam BV
             * @param model
             * @param radius
             * @param offset
             */
            template<class BV, typename T> static void createSphere(std::shared_ptr<::fcl::BVHModel<BV> > model, T radius, const fcl::Transform3<T> &offset) {
                generateBVHModel(*model, fcl::Sphere<T>(radius), offset, 6, 6, fcl::FinalizeModel::DONT);
            }
        }
    }
}

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <map>
#include <stack>
#include <iostream>
#include <memory>
#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <exception>
#include <vector>
#include "DynamicsLib/Pseudoinversion.h"

#include <fcl/fcl.h>
// #include <fcl/math/vec_3f.h>
// #include <fcl/math/matrix_3f.h>
// #include <fcl/math/transform.h>
// #include <fcl/collision_object.h>

// #include <fcl/collision_data.h>
// #include <fcl/collision_node.h>
// #include <fcl/collision.h>
// #include <fcl/BVH/BVH_model.h>
// #include <fcl/shape/geometric_shapes.h>
// #include <fcl/shape/geometric_shape_to_BVH_model.h>

#include <random>
#include <spdlog/spdlog.h>
#include <fmt/ostream.h>

#include <type_traits>
#include "DynamicsLib/typedefs.hpp"

namespace DL {
    namespace util {
        class logger {
        private:
            #define DEFER1(m) m EMPTY()
            std::shared_ptr<spdlog::logger> console;
        public:
            logger(std::string name) {
                console = spdlog::get(name);
                if (!console) {
                    console = spdlog::stdout_color_mt(name);
                }
                #ifdef LOG_DEBUG_ON
                spdlog::set_level(spdlog::level::debug);
                #endif
            }
            std::shared_ptr<spdlog::logger> getConsole() {
                return console;
            }

            #ifdef LOG_DEBUG_ON
                #if LOG_DEBUG_ON >= 1
                #define LOG_DEBUG_LEVEL1(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 1) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL1]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL1(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 2
                #define LOG_DEBUG_LEVEL2(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 2) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL2]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL2(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 3
                #define LOG_DEBUG_LEVEL3(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 3) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL3]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL3(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 4
                #define LOG_DEBUG_LEVEL4(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 4) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL4]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL4(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 5
                #define LOG_DEBUG_LEVEL5(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 5) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL5]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL5(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 6
                #define LOG_DEBUG_LEVEL6(INPUT, ...) (DL::util::RUNTIME_LOG_LEVEL < 6) ? (void)0 : (void)DL::util::console->getConsole()->debug((std::string("\033[33m[LEVEL6]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
                #define LOG_DEBUG_LEVEL6(INPUT, ...) (void)0
                #endif
            #else
                #define LOG_DEBUG_LEVEL1(INPUT, ...) (void)0
                #define LOG_DEBUG_LEVEL2(INPUT, ...) (void)0
                #define LOG_DEBUG_LEVEL3(INPUT, ...) (void)0
                #define LOG_DEBUG_LEVEL4(INPUT, ...) (void)0
                #define LOG_DEBUG_LEVEL5(INPUT, ...) (void)0
                #define LOG_DEBUG_LEVEL6(INPUT, ...) (void)0
            #endif
        };

        static std::random_device rd;
        static std::mt19937 e2(rd());
        static std::uniform_real_distribution<> rand_dist(0, 1);
        static auto console = std::make_shared<logger>("console");
        #ifdef LOG_DEBUG_ON
        static size_t RUNTIME_LOG_LEVEL = LOG_DEBUG_ON;
        #endif

        template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> &skewMat) {
            skewMat.setZero();
            skewMat(2, 1) = vector(0);
            skewMat(1, 2) = -vector(0);
            skewMat(0, 2) = vector(1);
            skewMat(2, 0) = -vector(1);
            skewMat(0, 1) = -vector(2);
            skewMat(1, 0) = vector(2);
        }

        template<typename Derived, typename Derived2> static void skewMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &skewMat) {
            typedef typename Derived::Scalar Scalar;
            const_cast< Eigen::MatrixBase<Derived2> & >(skewMat).setZero();
            const_cast<Scalar &>(skewMat(2, 1)) = vector(0);
            const_cast<Scalar &>(skewMat(1, 2)) = -vector(0);
            const_cast<Scalar &>(skewMat(0, 2)) = vector(1);
            const_cast<Scalar &>(skewMat(2, 0)) = -vector(1);
            const_cast<Scalar &>(skewMat(0, 1)) = -vector(2);
            const_cast<Scalar &>(skewMat(1, 0)) = vector(2);
        }

        template<typename Derived, typename Derived2> static void tildeMatrix(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &skewMat) {
            typedef typename Derived::Scalar Scalar;
            const_cast< Eigen::MatrixBase<Derived2> & >(skewMat).setZero();
            const_cast<Scalar &>(skewMat(0, 0)) = vector(0);
            const_cast<Scalar &>(skewMat(1, 1)) = vector(1);
            const_cast<Scalar &>(skewMat(2, 2)) = vector(2);
            const_cast<Scalar &>(skewMat(1, 3)) = vector(0);
            const_cast<Scalar &>(skewMat(2, 4)) = vector(0);
            const_cast<Scalar &>(skewMat(0, 3)) = vector(1);
            const_cast<Scalar &>(skewMat(2, 5)) = vector(1);
            const_cast<Scalar &>(skewMat(0, 4)) = vector(2);
            const_cast<Scalar &>(skewMat(1, 5)) = vector(2);
        }

        template<typename Derived, typename Derived2> static void momentumRegressor(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &regressorMat) {
            const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).setZero();
            const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 1>(0, 0) = vector.template head<3>();
            skewMatrix(vector.template tail<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 3>(0, 1));
            skewMatrix(-vector.template head<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 3>(3, 1));
            tildeMatrix(vector.template tail<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 6>(3, 4));
        }

        template<typename Derived, typename Derived2> static void frictionRegressor(const Eigen::MatrixBase<Derived> &qd, Eigen::MatrixBase<Derived2> const &out) {
            const_cast< Eigen::MatrixBase<Derived2> & >(out).setZero();
            for (int i = 0; i < qd.rows(); i++) {
                const_cast< Eigen::MatrixBase<Derived2> & >(out)(i, i) = qd;
                const_cast< Eigen::MatrixBase<Derived2> & >(out)(i, i + qd.rows()) = sgn(qd, 1e-30f);
            }
        }

        template<typename Derived, typename Derived2> static void
        unskewMatrix(const Eigen::MatrixBase<Derived> &skewMat, Eigen::MatrixBase<Derived2> const &vector) {
            typedef typename Derived::Scalar Scalar;
            const_cast< Eigen::MatrixBase<Derived2> & >(vector).setZero();
            const_cast<Scalar &>(vector(0)) = 0.5 * (skewMat(2, 1) - skewMat(1, 2));
            const_cast<Scalar &>(vector(1)) = 0.5 * (skewMat(0, 2) - skewMat(2, 0));
            const_cast<Scalar &>(vector(2)) = 0.5 * (skewMat(1, 0) - skewMat(0, 1));
        }

        template<typename Derived, typename Derived2, typename F> static std::stack<Derived> searchBreadth(Derived root, F addChildren, Derived2 target) {
            std::stack<Derived> output;
            std::queue<Derived> searchQueue;
            std::map<Derived, Derived> paths;
            searchQueue.push(root);
            paths.insert(std::pair<Derived, Derived>(root, root));
            while (!searchQueue.empty()) {
                if (!(searchQueue.front()->name == target)) {
                    addChildren(searchQueue.front(), searchQueue, paths);
                    searchQueue.pop();
                } else {
                    Derived temp = searchQueue.front();
                    while (!(paths.at(temp) == temp)) {
                        output.push(temp);
                        temp = paths.at(temp);
                    }

                    output.push(temp);
                    break;
                }
            }

            return output;
        }

        template <typename T> static int sgn(const T &val, const T &eps = T(1e-50)) {
            return val / std::sqrt(std::pow(val, 2) + eps);
        }
        template <typename T> static T sgnDeriv(const T &val, const T &eps = T(1e-50)) {
            return eps / std::pow(std::pow(val, 2) + eps, 1.5);
        }
        template <typename T> static int sgnIF(const T &val, const T &eps = T(0)) {
            return (T(eps) < val) - (val < T(-eps));
        }
        template <typename T> static int limit(const T &val, const T &eps = T(0)) {
            return (T(eps) < val) + (val < T(-eps));
        }
        template<typename T> static constexpr
        T const &constexpr_max(T const &a, T const &b) {
            return a > b ? a : b;
        }

        template<class BV> static void createMesh(std::shared_ptr<::fcl::BVHModel<BV> > model, Eigen::Vector3f scale, std::string &filename, const fcl::Transform3f &offset) {
            aiPropertyStore *propertyStore = aiCreatePropertyStore();
            aiSetImportPropertyInteger(propertyStore,
                                       AI_CONFIG_PP_SBP_REMOVE,
                                       aiPrimitiveType_POINT
                                       | aiPrimitiveType_LINE);

            std::string extension;
            const std::size_t extensionIndex = filename.find_last_of('.');
            if (extensionIndex != std::string::npos)
                extension = filename.substr(extensionIndex + 1);
            std::transform(std::begin(extension), std::end(extension),
                           std::begin(extension), ::tolower);
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
            std::ifstream t(filename);
            std::string input_model((std::istreambuf_iterator<char>(t)),
                                    std::istreambuf_iterator<char>());

            std::cout << "input mesh file: " << input_model << std::endl;

            const aiScene *_mesh = aiImportFileFromMemoryWithProperties(input_model.c_str(), input_model.size(), aiProcess_GenNormals
                                                                        | aiProcess_Triangulate
                                                                        | aiProcess_JoinIdenticalVertices
                                                                        | aiProcess_SortByPType
                                                                        | aiProcess_OptimizeMeshes, extension.c_str(), propertyStore);
            // std::cout << "LOADED MESH FROM FILE" << std::endl;
            if (!_mesh) {
                throw std::runtime_error("MODEL NOT LOADED!!!");
            }

            if (extension == "dae" || extension == "zae")
                _mesh->mRootNode->mTransformation = aiMatrix4x4();

            // Finally, pre-transform the vertices. We can't do this as part of the
            // import process, because we may have changed mTransformation above.
            _mesh = aiApplyPostProcessing(_mesh, aiProcess_PreTransformVertices);
            // std::cout << "POSTPROCESS" << std::endl;
            if (!_mesh)
                throw std::runtime_error("MODEL NOT LOADED!!!");

            assert(_mesh);
            // std::shared_ptr<::fcl::BVHModel<BV> > model;
            // model.reset(new ::fcl::BVHModel<BV>());
            if (model->build_state == fcl::BVH_BUILD_STATE_EMPTY) {
                model->beginModel();
            }
            for (std::size_t i = 0; i < _mesh->mNumMeshes; i++) {
                for (std::size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++) {
                    Eigen::Vector3f vertices[3];
                    for (std::size_t k = 0; k < 3; k++) {
                        const aiVector3D &vertex =
                            _mesh->mMeshes[i]->mVertices[
                                _mesh->mMeshes[i]->mFaces[j].mIndices[k]];
                        vertices[k] = Eigen::Vector3f(vertex.x * scale[0],
                                                      vertex.y * scale[1],
                                                      vertex.z * scale[2]);
                        vertices[k] = offset * vertices[k];
                    }
                    model->addTriangle(vertices[0], vertices[1], vertices[2]);
                }
            }
        }

        template<class BV> static void createCylinder(std::shared_ptr<::fcl::BVHModel<BV> > model, float radius, float length, const fcl::Transform3f &offset) {
            // std::shared_ptr<::fcl::BVHModel<BV> > model;
            // model.reset(new ::fcl::BVHModel<BV>());
            generateBVHModel(*model, fcl::Cylinder<float>(radius, length), offset, 128, fcl::FinalizeModel::DONT);
            std::cout << "CYLINDER!!!" << std::endl;
            std::cout << model->num_vertices << std::endl;
            std::cout << model->num_tris << std::endl;
        }
        template<class BV> static void createBox(std::shared_ptr<::fcl::BVHModel<BV> > model, const Eigen::Vector3f &size, const fcl::Transform3f &offset) {
            // std::shared_ptr<::fcl::BVHModel<BV> > model;
            // model.reset(new ::fcl::BVHModel<BV>());
            generateBVHModel(*model, fcl::Box<float>(size), offset, fcl::FinalizeModel::DONT);
            std::cout << "BOX!!!" << std::endl;
            std::cout << model->num_vertices << std::endl;
            std::cout << model->num_tris << std::endl;
        }
        template<class BV> static void createSphere(std::shared_ptr<::fcl::BVHModel<BV> > model, float radius, const fcl::Transform3f &offset) {
            // std::shared_ptr<::fcl::BVHModel<BV> > model;
            // model.reset(new ::fcl::BVHModel<BV>());
            generateBVHModel(*model, fcl::Sphere<float>(radius), offset, 6, 6, fcl::FinalizeModel::DONT);
            std::cout << "SPHERE!!!!6,6" << std::endl;
            std::cout << model->num_vertices << std::endl;
            std::cout << model->num_tris << std::endl;
        }

        template <typename T> static void PluckerToHomogenous(Eigen::Matrix<T, 6, 6> &p, Eigen::Matrix<T, 4, 4> &h) {
            h.template setIdentity();
            h.template block<3, 3>(0, 0) = p.template block<3, 3>(0, 0);
            unskewMatrix((p.template block<3, 3>(3, 0) * p.template block<3, 3>(0, 0).transpose()), h.template block<3, 1>(0, 3));
        }

        template <typename T> static void HomogenousToPlucker(Eigen::Matrix<T, 4, 4> &h, Eigen::Matrix<T, 6, 6> &p) {
            // //util::logger->debug("File: Util Message: HTP setZero");
            p.template setZero();
            // //util::logger->debug("File: Util Message: HTP block block block");
            p.template block<3, 3>(0, 0) = p.template block<3, 3>(3, 3) = h.template block<3, 3>(0, 0).transpose();
            // //util::logger->debug("File: Util Message: HTP skewMat");
            skewMatrix(h.template block<3, 1>(0, 3), p.template block<3, 3>(3, 0));
            // //util::logger->debug( "File: Util Message: HTP block {} \n {}",p.block<3,3>(3,0), p.block<3,3>(0,0) );
            p.template block<3, 3>(3, 0) = -(p.template block<3, 3>(0, 0) * p.template block<3, 3>(3, 0));
            // //util::logger->debug("File: Util Message: HTP end");
        }

        template <typename T> static void unrotatePlucker(Eigen::Matrix<T, 6, 6> &p) {
            p.template block<3, 3>(3, 0) = p.template block<3, 3>(0, 0).transpose() * p.template block<3, 3>(3, 0);
            p.template block<3, 3>(0, 0).template setIdentity();
            p.template block<3, 3>(3, 3).template setIdentity();
        }

        template <typename T> static void unrotatePlucker(Eigen::Matrix<T, 6, 6> &p, Eigen::Matrix<T, 6, 6> &h) {
            h.template block<3, 3>(3, 0) = p.template block<3, 3>(0, 0).transpose() * p.template block<3, 3>(3, 0);
            h.template block<3, 3>(0, 0).template setIdentity();
            h.template block<3, 3>(3, 3).template setIdentity();
        }

        template <typename T> static Eigen::Matrix<T, 6, 6> PluckerInverse(const Eigen::Matrix<T, 6, 6> &in) {
            Eigen::Matrix<T, 6, 6> out;
            out.template setZero();
            out.template block<3, 3>(0, 0) = in.template block<3, 3>(0, 0).transpose();
            out.template block<3, 3>(3, 3) = in.template block<3, 3>(3, 3).transpose();
            out.template block<3, 3>(3, 0) = -in.template block<3, 3>(3, 3).transpose() * (in.template block<3, 3>(3, 0) * in.template block<3, 3>(3, 3).transpose());

            return out;
        }

        template <typename T> static void HomogenousToAd(Eigen::Matrix<T, 4, 4> &h, Eigen::Matrix<T, 6, 6> &p) {
            // //util::logger->debug("File: Util Message: HTP setZero");
            p.template setZero();
            // //util::logger->debug("File: Util Message: HTP block block block");
            p.template block<3, 3>(0, 0) = p.template block<3, 3>(3, 3) = h.block<3, 3>(0, 0);
            // //util::logger->debug("File: Util Message: HTP skewMat");
            skewMatrix(h.template block<3, 1>(0, 3), p.template block<3, 3>(3, 0));
            // //util::logger->debug( "File: Util Message: HTP block {} \n {}",p.block<3,3>(3,0), p.block<3,3>(0,0) );
            p.template block<3, 3>(3, 0) = p.template block<3, 3>(3, 0) * p.template block<3, 3>(0, 0);
            // //util::logger->debug("File: Util Message: HTP end");
        }

        template <typename T> static void TwistToAdj(Eigen::Matrix<T, 6, 1> &t, Eigen::Matrix<T, 6, 6> &p) {
            // //util::logger->debug("File: Util Message: HTP setZero");
            p.setZero();
            // //util::logger->debug("File: Util Message: HTP block block block");
            skewMatrix(t.block<3, 1>(3, 0), p.block<3, 3>(3, 3));
            p.block<3, 3>(0, 0) = p.block<3, 3>(3, 3);
            // //util::logger->debug("File: Util Message: HTP skewMat");
            skewMatrix(t.block<3, 1>(0, 0), p.block<3, 3>(3, 0));
            // //util::logger->debug( "File: Util Message: HTP block {} \n {}",p.block<3,3>(3,0), p.block<3,3>(0,0) );
            // //util::logger->debug("File: Util Message: HTP end");
        }
        template <typename T> class DiscreteLowPassFilter {
        public:
            DiscreteLowPassFilter(T cutoffFrequency, T samplingRate, size_t rows, size_t cols) : DiscreteLowPassFilter<T>(cutoffFrequency, samplingRate) {
                buffer.resize(rows, cols);
                initial.resize(rows, cols);
            }
            DiscreteLowPassFilter(T cutoffFrequency, T samplingRate, size_t size) : DiscreteLowPassFilter<T>(cutoffFrequency, samplingRate) {
                buffer.resize(size, size);
                initial.resize(size, size);
            }
            DiscreteLowPassFilter(T cutoffFrequency, T samplingRate) {
                T tmp1 = 1 / (cutoffFrequency * 2 * M_PI);
                T tmp2 = 1 / samplingRate;

                gamma = tmp2 / (tmp1 + tmp2);
                samplingTime = 1.0 / samplingRate;
            }
            template <typename Derived> Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> compute(const Eigen::MatrixBase<Derived> &newInput) {
                static_assert(std::is_same<typename Derived::Scalar, T>::value, "must be same scalar");
                buffer = (1 - gamma) * newInput + gamma * buffer;
                return buffer;
            }
            template <typename Derived> void setBuffer(const Eigen::MatrixBase<Derived> &buf) {
                buffer = buf;
                initial = buf;
            }
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getResult() {
                return buffer;
            }
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getInitial() {
                return initial;
            }
            void setCutoffAndSampling(T &cutoff, T &sampling) {
                T tmp1 = 1 / (cutoff * 2 * M_PI);
                T tmp2 = 1 / sampling;

                gamma = tmp2 / (tmp1 + tmp2);
                std::cout << "GAMMA DL: " << gamma << std::endl;
                samplingTime = 1.0 / sampling;
            }
            T getBeta() {
                return (1 - gamma) * (1 / gamma) * (1 / samplingTime);
            }
        private:
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> buffer;
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> initial;
            T gamma;
            T samplingTime;
        };
    }
}

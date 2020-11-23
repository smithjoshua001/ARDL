#pragma once
#include <Eigen/Dense>
#include <cstddef>
#include <string>
#include "ARDL/Model/Joints/Joints.hpp"
#include "ARDL/Model/Link.hpp"
using namespace ARDL::Model;
namespace ARDL {
    template<typename T>
    struct Data {
        Data(std::string modelPath):Data(urdf::parseURDFFile(modelPath)){

        }
        Data(urdf::ModelInterfaceSharedPtr model){
            std::vector<urdf::LinkSharedPtr> links;
            model->getLinks(links);
            std::cout<<"MODEL HAS "<< links.size()<<" Links" <<std::endl;
            reserve(links.size());
        }
        //--------------------Chain Data------------------------

        // List of links (all/movable)
        aligned_vector<Link<T>> links;
        aligned_vector<Link<T>*> movable_links;
        // List of movable joints
        aligned_vector<JointVariant<T>> joints_all;
        aligned_vector<JointVariant<T>*> joints;
        void reserve(size_t size){
            links.reserve(size);
            movable_links.reserve(size);
            joints_all.reserve(size);
            joints.reserve(size);
            reserveLinks(size);
            reserveJoints(size);
        }

        // Number of movable joints
        size_t jointNum;
        // Root name of chain and tip name of chain
        std::string rootName, tipName;

        // current position and velocity states of the chain
        Eigen::Matrix<T, Eigen::Dynamic, 1> q, qd;

        //---------------------Link Data--------------------------

        std::vector<char> isRoot, hasInertial, hasCollision;
        std::vector<std::string> name;

        aligned_vector<Eigen::Matrix<T, 12, 1>> inertialParameters;

        std::vector<std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>> collisionModel_sp, collisionModelOptimal_sp;
        std::vector<std::shared_ptr<fcl::CollisionObject<T>>> collisionObject_sp;
        std::vector<std::vector<std::string>> collisionFiles;

        aligned_vector<SpatialInertia<T>> sI;

        void reserveLinks(size_t size){
            isRoot.reserve(size);
            hasInertial.reserve(size);
            hasCollision.reserve(size);
            name.reserve(size);
            inertialParameters.reserve(size);
            collisionModel_sp.reserve(size);
            collisionModelOptimal_sp.reserve(size);
            collisionObject_sp.reserve(size);
            collisionFiles.reserve(size);
            sI.reserve(size);
        }

        void addLink(){
            isRoot.push_back(false);
            hasInertial.push_back(false);
            hasCollision.push_back(false);
            name.push_back("");
            inertialParameters.push_back(Eigen::Matrix<T, 12, 1>::Zero());
            collisionModel_sp.push_back(nullptr);
            collisionModelOptimal_sp.push_back(nullptr);
            collisionObject_sp.push_back(nullptr);
            collisionFiles.push_back(std::vector<std::string>());
            sI.push_back(SpatialInertia<T>());
        }

        //---------------------Joint Data--------------------------
        aligned_vector<Eigen::Matrix<T, 6, 1>> s;
        // Optimized Transform to origin
        aligned_vector<Pose<T>> optimOriginTransform;

        // transform to origin
        aligned_vector<Pose<T>> originTransform;
        // transform from origin to end of joint
        aligned_vector<Pose<T>> jointTransform;

        // Velocity of Joint
        aligned_vector<Motion<T>> adjPK;

        // Transformation of Joint
        aligned_vector<Pose<T>> adPK, adPKOptim;

        std::vector<T> staticFriction;
        std::vector<T> viscousFriction;

        aligned_vector<VectorX<T>> qMin, qMax, qdLimit, j_q, j_qd;

        std::vector<char> cache, qd_cache;
         void reserveJoints(size_t size){
            s.reserve(size);
            // if(!fixed){
            optimOriginTransform.reserve(size);
            adPKOptim.reserve(size);
            // }
            originTransform.reserve(size);
            jointTransform.reserve(size);
            adjPK.reserve(size);
            adPK.reserve(size);
            staticFriction.reserve(size);
            viscousFriction.reserve(size);
            qMin.reserve(size);
            qMax.reserve(size);
            qdLimit.reserve(size);
            j_q.reserve(size);
            j_qd.reserve(size);
            cache.reserve(size);
            qd_cache.reserve(size);
        }
        void addJoint(bool fixed, size_t dof){
            s.push_back(Eigen::Matrix<T, 6, 1>::Zero());
            // if(!fixed){
            optimOriginTransform.push_back(Pose<T>());
            adPKOptim.push_back(Pose<T>());
            // }
            originTransform.push_back(Pose<T>());
            jointTransform.push_back(Pose<T>());
            adjPK.push_back(Motion<T>());
            adPK.push_back(Pose<T>());
            staticFriction.push_back(T(0));
            viscousFriction.push_back(T(0));
            qMin.push_back(VectorX<T>(dof));
            qMax.push_back(VectorX<T>(dof));
            qdLimit.push_back(VectorX<T>(dof));
            j_q.push_back(VectorX<T>(dof));
            j_qd.push_back(VectorX<T>(dof));
            cache.push_back(false);
            qd_cache.push_back(false);
        }
    };
} // namespace ARDL
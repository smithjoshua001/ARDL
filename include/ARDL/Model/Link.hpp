#pragma once
#include <string>
#include <Eigen/Dense>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <array>
#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include "ARDL/Util/Math.hpp"
#include "ARDL/Util/Collision.hpp"
#include "ARDL/Model/Joint.hpp"
#include "ARDL/Model/Joints/Joints.hpp"
#include "ARDL/typedefs.hpp"
#include "ARDL/Math/SpatialInertia.hpp"

namespace ARDL {
    using namespace Util::Math;
    using namespace Util::Collision;
    namespace Model {
        using namespace Joints;
        template<typename T>
        class Link
            : public std::enable_shared_from_this<Link<T>>
        {
             private:
            bool m_isRoot, m_hasInertial, m_hasCollision;
            std::string m_name;

            Eigen::Matrix<T, 12, 1> m_inertialParameters;

            JointVariant<T> *m_parentJoint_p;
            Link<T> *m_parentLink_p;

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>> m_collisionModel_sp, m_collisionModelOptimal_sp;
            std::shared_ptr<fcl::CollisionObject<T>> m_collisionObject_sp, m_collisionObjectOptimal_sp;
            std::vector<std::string> m_collisionFiles;

            SpatialInertia<T> m_sI;

            urdf::LinkConstSharedPtr urdfModel;

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Link(const Link<T> &copy, JointVariant<T> *parentJoint, Link<T> *parentLink)
            {
                m_isRoot= copy.m_isRoot;
                m_hasInertial= copy.m_hasInertial;
                m_hasCollision= copy.m_hasCollision;

                m_name= copy.m_name;
                if(m_hasInertial) {
                    m_inertialParameters= copy.m_inertialParameters;
                    m_sI= copy.m_sI;
                }

                m_parentLink_p= parentLink;
                m_parentJoint_p= parentJoint;
                if(m_hasCollision) {
                    m_collisionModel_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                        new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModel_sp)));
                    if(copy.m_collisionModelOptimal_sp) {
                        m_collisionModelOptimal_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                            new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModelOptimal_sp)));
                    }
                    m_collisionObject_sp= std::shared_ptr<fcl::CollisionObject<T>>(
                        new fcl::CollisionObject<T>(*(copy.m_collisionObject_sp)));
                    if(copy.m_collisionModelOptimal_sp) {
                        m_collisionObjectOptimal_sp= std::shared_ptr<fcl::CollisionObject<T>>(
                            new fcl::CollisionObject<T>(*(copy.m_collisionObjectOptimal_sp)));
                    }
                    m_collisionFiles= copy.m_collisionFiles;
                }
                this->urdfModel= copy.urdfModel;
            }

            /***************** INITIALIZATION HANDLING *****************/

            Link(urdf::LinkConstSharedPtr link, Link<T> *parentLink, JointVariant<T> *parentJoint)
                : urdfModel(link)
            {
                m_name= link->name;
                m_isRoot= (parentLink == nullptr);
                m_parentLink_p= parentLink;
                m_parentJoint_p= parentJoint;

                    m_hasInertial= true;
                if(link->inertial) {
                    m_inertialParameters.setZero();
                    m_inertialParameters(0)= link->inertial->mass;

                    Eigen::Quaterniond t_quaternion;
                    link->inertial->origin.rotation.getQuaternion(t_quaternion.x(), t_quaternion.y(), t_quaternion.z(),
                                                                  t_quaternion.w());

                    Eigen::Matrix<T, 3, 3> t_rotationCOM= t_quaternion.toRotationMatrix().cast<T>();
                    t_rotationCOM.setIdentity();

                    Eigen::Matrix<T, 3, 1> t_translationCOM;
                    t_translationCOM << link->inertial->origin.position.x, link->inertial->origin.position.y,
                        link->inertial->origin.position.z;

                    m_inertialParameters.template segment<3>(1)= m_inertialParameters(0) * t_translationCOM;

                    Eigen::Matrix<T, 3, 3> t_rotInertiaWRTOrigin;
                    t_rotInertiaWRTOrigin(0, 0)= link->inertial->ixx;
                    t_rotInertiaWRTOrigin(1, 1)= link->inertial->iyy;
                    t_rotInertiaWRTOrigin(2, 2)= link->inertial->izz;
                    t_rotInertiaWRTOrigin(0, 1)= t_rotInertiaWRTOrigin(1, 0)= link->inertial->ixy;
                    t_rotInertiaWRTOrigin(0, 2)= t_rotInertiaWRTOrigin(2, 0)= link->inertial->ixz;
                    t_rotInertiaWRTOrigin(1, 2)= t_rotInertiaWRTOrigin(2, 1)= link->inertial->iyz;

                    Eigen::Matrix<T, 3, 3> t_rotInertiaWRTCOM= t_rotationCOM * t_rotInertiaWRTOrigin;

                    Eigen::Matrix<T, 3, 3> t_translationCOMSkew;
                    t_translationCOMSkew.setZero();
                    skewMatrix(t_translationCOM, t_translationCOMSkew);

                    t_rotInertiaWRTCOM= t_rotInertiaWRTCOM + (t_translationCOMSkew * t_translationCOMSkew.transpose() *
                                                              m_inertialParameters(0));

                    m_inertialParameters(4)= t_rotInertiaWRTCOM(0, 0); // xx
                    m_inertialParameters(5)= t_rotInertiaWRTCOM(1, 1); // yy
                    m_inertialParameters(6)= t_rotInertiaWRTCOM(2, 2); // zz
                    m_inertialParameters(7)= t_rotInertiaWRTCOM(0, 1); // xy
                    m_inertialParameters(8)= t_rotInertiaWRTCOM(0, 2); // xz
                    m_inertialParameters(9)= t_rotInertiaWRTCOM(1, 2); // yz
                    m_sI= SpatialInertia<T>(m_inertialParameters);
                }
                else{
                    m_inertialParameters.setZero();
                    m_sI =  SpatialInertia<T>(m_inertialParameters);
                }

                if(link->collision) {
                    m_hasCollision= true;
                    // If made of multiple collision primitives
                    if(link->collision_array.size() > 1) {
                        m_collisionModel_sp=
                            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(new fcl::BVHModel<fcl::OBB<T>>());
                        for(size_t i= 0; i < link->collision_array.size(); i++) {
                            loadCollision(link->collision_array[i]);
                        }
                        m_collisionModel_sp->endModel();
                        m_collisionModel_sp->computeLocalAABB();
                    } else {
                        // made of a single collision object
                        m_collisionModel_sp=
                            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(new fcl::BVHModel<fcl::OBB<T>>());
                        loadCollision(link->collision);

                        m_collisionModel_sp->endModel();
                        m_collisionModel_sp->computeLocalAABB();
                    }
                    m_collisionObject_sp.reset(new fcl::CollisionObject<T>(m_collisionModel_sp));
                    if(!m_isRoot && m_parentLink_p->m_hasCollision &&
                       m_parentLink_p->m_collisionModelOptimal_sp != nullptr &&
                       ARDL_visit(*m_parentJoint_p, isFixed())) {
                        m_collisionModelOptimal_sp=
                            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(new fcl::BVHModel<fcl::OBB<T>>());
                        std::vector<Matrix<T, 3, 1>> ps;
                        std::vector<fcl::Triangle> ts;

                        Pose transform(ARDL_visit(*m_parentJoint_p, getOriginTransform()));
                        fcl::Transform3<T> t_collisionOffset;
                        t_collisionOffset.translation()= transform.getP();
                        t_collisionOffset.linear()= transform.getR();
                        for(size_t i= 0; i < m_collisionModel_sp->num_vertices; i++) {
                            ps.push_back(t_collisionOffset * (m_collisionModel_sp->vertices)[i]);
                        }
                        for(size_t i= 0; i < m_collisionModel_sp->num_tris; i++) {
                            ts.push_back(m_collisionModel_sp->tri_indices[i]);
                        }
                        for(size_t j= 0; j < m_parentLink_p->m_collisionModelOptimal_sp->num_vertices; j++) {
                            ps.push_back((m_parentLink_p->m_collisionModelOptimal_sp->vertices)[j]);
                        }
                        for(size_t j= 0; j < m_parentLink_p->m_collisionModelOptimal_sp->num_tris; j++) {
                            ts.push_back(m_parentLink_p->m_collisionModelOptimal_sp->tri_indices[j]);
                            ts.back()[0]+= m_collisionModel_sp->num_vertices;
                            ts.back()[1]+= m_collisionModel_sp->num_vertices;
                            ts.back()[2]+= m_collisionModel_sp->num_vertices;
                        }

                        m_collisionModelOptimal_sp->beginModel();
                        if(m_collisionModelOptimal_sp->addSubModel(ps, ts) != fcl::BVH_OK) {
                            std::cerr << "COLLISION FAIL" << std::endl;
                        };
                        m_collisionModelOptimal_sp->endModel();
                        m_collisionModelOptimal_sp->computeLocalAABB();
                        m_parentLink_p->m_collisionModelOptimal_sp= m_collisionModelOptimal_sp;
                        m_collisionModelOptimal_sp= nullptr;
                        m_parentLink_p->m_collisionModelOptimal_sp->computeLocalAABB();

                        m_parentLink_p->m_collisionObjectOptimal_sp.reset(
                            new fcl::CollisionObject<T>(m_parentLink_p->m_collisionModelOptimal_sp));
                    } else if(m_hasCollision) {
                        m_collisionModelOptimal_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                            new fcl::BVHModel<fcl::OBB<T>>(*(m_collisionModel_sp)));
                        m_collisionObjectOptimal_sp.reset(new fcl::CollisionObject<T>(m_collisionModelOptimal_sp));
                    }
                }
                if(!m_isRoot) {
                    if(!ARDL_visit(*m_parentJoint_p, isFixed())) {
                        m_inertialParameters[10]= T(ARDL_visit(*m_parentJoint_p, getViscousFriction()));
                        m_inertialParameters[11]= T(ARDL_visit(*m_parentJoint_p, getStaticFriction()));
                    }
                    if(ARDL_visit(*m_parentJoint_p, isFixed()) && !m_parentLink_p->isRoot()) {
                        Pose<T> t_fixedTransform= ARDL_visit(*m_parentJoint_p, getOriginTransform());
                        Link<T> *parentL= m_parentLink_p;
                        while(ARDL_visit(*(parentL->m_parentJoint_p), isFixed())) {
                            t_fixedTransform.apply(ARDL_visit(*(parentL->m_parentJoint_p), getOriginTransform()));
                            parentL= parentL->m_parentLink_p;
                        }
                        if(m_hasInertial) {
                            m_sI.applyInverseXIX(t_fixedTransform);
                            parentL->m_sI+= m_sI;
                            // parentL->m_sI.applyXIX(t_fixedTransform);
                            // m_sI = parentL->m_sI;
                        }
                    } 
                }
            }
            Link(urdf::LinkConstSharedPtr link): Link(link, nullptr, nullptr){};

            ~Link() {}

            void loadCollision(const urdf::CollisionSharedPtr &collision) {
                fcl::Transform3<T> t_collisionOffset;
                t_collisionOffset.setIdentity();
                t_collisionOffset.translation()=
                    Eigen::Vector3f(collision->origin.position.x, collision->origin.position.y,
                                    collision->origin.position.z)
                        .cast<T>();
                Eigen::Quaterniond quat;
                collision->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                t_collisionOffset.linear()= quat.cast<T>().matrix();

                std::shared_ptr<urdf::Mesh> t_mesh_sp= std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
                if(t_mesh_sp) {
                    Eigen::Matrix<T, 3, 1> t_scale(t_mesh_sp->scale.x, t_mesh_sp->scale.y, t_mesh_sp->scale.z);
                    m_collisionFiles.push_back(t_mesh_sp->filename);
                    createMesh<fcl::OBB<T>>(m_collisionModel_sp, t_scale, t_mesh_sp->filename, t_collisionOffset);
                } else {
                    std::shared_ptr<urdf::Cylinder> cylinder=
                        std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
                    if(cylinder) {
                        createCylinder<fcl::OBB<T>>(m_collisionModel_sp, (T) cylinder->radius, (T) cylinder->length,
                                                    t_collisionOffset);
                        m_collisionFiles.push_back("CYLINDER");
                    } else {
                        std::shared_ptr<urdf::Box> box= std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
                        if(box) {
                            Eigen::Matrix<T, 3, 1> scale;
                            scale << (T) box->dim.x, (T) box->dim.y, (T) box->dim.z;
                            createBox<fcl::OBB<T>>(m_collisionModel_sp, scale, t_collisionOffset);
                            m_collisionFiles.push_back("BOX");
                        } else {
                            std::shared_ptr<urdf::Sphere> sphere=
                                std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                            if(sphere) {
                                createSphere<fcl::OBB<T>>(m_collisionModel_sp, (T) sphere->radius, t_collisionOffset);
                                m_collisionFiles.push_back("SPHERE");
                            }
                        }
                    }
                }
            }

            void loadChildJoint(std::string childName, aligned_vector<JointVariant<T>> &joints_all_vsp)
            {
                for(urdf::JointSharedPtr joint: urdfModel->child_joints) {
                    if(joint->child_link_name == childName) {
                        switch(joint->type) {
                        case urdf::Joint::REVOLUTE: joints_all_vsp.emplace_back(RevoluteJoint<T>(joint, *this)); break;
                        case urdf::Joint::PRISMATIC: joints_all_vsp.emplace_back(PrismaticJoint<T>(joint, *this));break;
                        case urdf::Joint::FIXED: joints_all_vsp.emplace_back(FixedJoint<T>(joint, *this)); break;
                        default: throw std::runtime_error("DEFAULT JOINT NOT IMPLEMENTED"); break;
                        }
                        break;
                    }
                }
            }

            /***************** END INITIALIZATION HANDLING *****************/

            SpatialInertia<T> &getSI() { return m_sI; }

            template<typename Derived>
            void setInertialParams(const Eigen::MatrixBase<Derived> &params) {
                m_sI= params;
            }
            JointVariant<T> &getParentJoint() const { return *m_parentJoint_p; }
            JointVariant<T> *getParentJointPtr() const { return m_parentJoint_p; }
            Link<T> &getParentLink() const { return *m_parentLink_p; }

            std::string const &getName() const { return m_name; }

            bool operator==(const Link<T> &rhs) { return this->m_name == rhs.getName(); }
            bool operator==(const std::string &rhs) { return this->m_name == rhs; }

            template<typename Derived>
            void updateParams(const Eigen::MatrixBase<Derived> &paramDot) {
                m_sI+= paramDot;
            }

            bool isRoot() { return m_isRoot; }
            

            /***************** COLLISION HANDLING *****************/
            /**
             * @brief Get the Collision Mesh File object
             *
             * @return std::string Filename if it exists otherwise returns an empty string
             */
            std::string getCollisionMeshFile() {
                if(m_hasCollision) {
                    return m_collisionFiles[0];
                } else {
                    return "";
                }
            }

            void updateCollision(const Pose<T> &at) {
                if(m_hasCollision) { updateCollision_us(at); }
            }
            void updateCollision_us(const Pose<T> &at) {
                m_collisionObject_sp->setTranslation(at.getP().template cast<T>());
                m_collisionObject_sp->setRotation(at.getR().template cast<T>());
            }
            void updateCollisionOptim(const Pose<T> &at) {
                if(m_hasCollision) { updateCollisionOptim_us(at); }
            }
            void updateCollisionOptim_us(const Pose<T> &at) {
                m_collisionObjectOptimal_sp->setTranslation(at.getP().template cast<T>());
                m_collisionObjectOptimal_sp->setRotation(at.getR().template cast<T>());
            }

            std::shared_ptr<fcl::CollisionObject<T>> getCollisionObjectPtr() { return m_collisionObject_sp; }
            std::shared_ptr<fcl::CollisionObject<T>> getCollisionObjectOptimPtr() {
                return m_collisionObjectOptimal_sp;
            }

            Eigen::Matrix<T, Eigen::Dynamic, 3> getCollisionVertices() {
                Eigen::Matrix<T, Eigen::Dynamic, 3> output;
                int vertices= m_collisionModel_sp->num_vertices;
                output.resize(vertices, 3);
                for(size_t i= 0; i < vertices; i++) { output.row(i)= m_collisionModel_sp->vertices[i].transpose(); }
                return output;
            }

            std::vector<std::array<int, 3>> getCollisionTriangles() {
                std::vector<std::array<int, 3>> output;
                int vertices= m_collisionModel_sp->num_tris;
                output.resize(vertices);
                for(size_t i= 0; i < vertices; i++) {
                    output[i][0]= m_collisionModel_sp->tri_indices[i][0];
                    output[i][1]= m_collisionModel_sp->tri_indices[i][1];
                    output[i][2]= m_collisionModel_sp->tri_indices[i][2];
                }
                return output;
            }

            Eigen::Matrix<T, Eigen::Dynamic, 3> getCollisionVerticesOptim() {
                Eigen::Matrix<T, Eigen::Dynamic, 3> output;
                int vertices= m_collisionModelOptimal_sp->num_vertices;
                output.resize(vertices, 3);
                for(size_t i= 0; i < vertices; i++) {
                    output.row(i)= m_collisionModelOptimal_sp->vertices[i].transpose();
                }
                return output;
            }

            std::vector<std::array<int, 3>> getCollisionTrianglesOptim() {
                std::vector<std::array<int, 3>> output;
                int vertices= m_collisionModelOptimal_sp->num_tris;
                output.resize(vertices);
                for(size_t i= 0; i < vertices; i++) {
                    output[i][0]= m_collisionModelOptimal_sp->tri_indices[i][0];
                    output[i][1]= m_collisionModelOptimal_sp->tri_indices[i][1];
                    output[i][2]= m_collisionModelOptimal_sp->tri_indices[i][2];
                }
                return output;
            }

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>> getCollisionModel() { return m_collisionModel_sp; }

            bool hasCollision() { return m_hasCollision; }

            /***************** END COLLISION HANDLING *****************/

            urdf::LinkConstSharedPtr getUrdfLink() const { return urdfModel; }
        };
    } // namespace Model
} // namespace ARDL

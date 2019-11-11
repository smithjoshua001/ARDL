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

namespace ARDL {
    using namespace Util::Math;
    using namespace Util::Collision;
    namespace Model {
        using namespace Joints;
        template<typename T> class Link : public std::enable_shared_from_this < Link<T> > {
        private:
            bool m_isRoot, m_hasInertial, m_hasCollision = false;
            std::string m_name;

            Eigen::Matrix<T, 12, 1> m_inertialParameters;

            std::shared_ptr<Joint<T> > m_parentJoint_sp;
            std::shared_ptr<Link<T> > m_parentLink_sp;

            std::map<std::string, std::shared_ptr<Joint<T> > > m_childJoints_sm;

            fcl::Transform3<T> mt_collisionOffset;

            const Eigen::Matrix<T, 3, 3> mt_identity3x3 = Eigen::Matrix<T, 3, 3>::Identity();

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T> > > m_collisionModel_sp;
            std::shared_ptr<fcl::CollisionObject<T> > m_collisionObject_sp;
            std::vector<std::string> m_collisionFiles;

            SpatialInertia<T> m_spatialInertia;

            urdf::LinkConstSharedPtr urdfModel;

            void calculateSpatialInertia() {
                skewMatrix(m_inertialParameters.template segment<3>(1), m_spatialInertia.template block<3, 3>(3, 0));
                m_spatialInertia.template block<3, 3>(0, 3) = -m_spatialInertia.template block<3, 3>(3, 0);
                m_spatialInertia(3, 3) = m_inertialParameters(4);
                m_spatialInertia(4, 4) = m_inertialParameters(5);
                m_spatialInertia(5, 5) = m_inertialParameters(6);
                m_spatialInertia(3, 4) = m_inertialParameters(7);
                m_spatialInertia(4, 3) = m_inertialParameters(7);
                m_spatialInertia(3, 5) = m_inertialParameters(8);
                m_spatialInertia(5, 3) = m_inertialParameters(8);
                m_spatialInertia(4, 5) = m_inertialParameters(9);
                m_spatialInertia(5, 4) = m_inertialParameters(9);

                m_spatialInertia.template block < 3, 3 > (0, 0) = m_inertialParameters(0) * mt_identity3x3;
            }

        public:

            Link(const Link<T> &copy, std::shared_ptr<Joint<T> > parentJoint, std::shared_ptr<Link<T> > parentLink) {
                m_isRoot = copy.m_isRoot;
                m_hasInertial = copy.m_hasInertial;
                m_hasCollision = copy.m_hasCollision;

                m_name = copy.m_name;
                if (m_hasInertial) {
                    m_inertialParameters = copy.m_inertialParameters;
                    m_spatialInertia = copy.m_spatialInertia;
                }
                m_parentJoint_sp = parentJoint;
                m_parentLink_sp = parentLink;
                if (m_hasCollision) {
                    mt_collisionOffset = copy.mt_collisionOffset;
                    m_collisionModel_sp = std::shared_ptr<fcl::BVHModel<fcl::OBB<T> > >(new fcl::BVHModel<fcl::OBB<T> >(*(copy.m_collisionModel_sp)));
                    m_collisionObject_sp = std::shared_ptr<fcl::CollisionObject<T> >(new fcl::CollisionObject<T>(*(copy.m_collisionObject_sp)));
                    m_collisionFiles = copy.m_collisionFiles;
                }
            }
            Link(const Link<T> &copy) = delete;

            /***************** INITIALIZATION HANDLING *****************/
            Link(urdf::LinkConstSharedPtr link, bool root, std::shared_ptr<Link<T> > parent_sp) : urdfModel(link) {
                m_isRoot = root;
                m_name = link->name;
                m_parentLink_sp = parent_sp;
                if (link->inertial) {
                    m_hasInertial = true;
                    m_inertialParameters.setZero();
                    m_inertialParameters(0) = link->inertial->mass;

                    Eigen::Quaterniond t_quaternion;
                    link->inertial->origin.rotation.getQuaternion(t_quaternion.x(), t_quaternion.y(), t_quaternion.z(), t_quaternion.w());

                    Eigen::Matrix<T, 3, 3> t_rotationCOM = t_quaternion.toRotationMatrix().cast<T>();

                    Eigen::Matrix<T, 3, 1> t_translationCOM;
                    t_translationCOM << link->inertial->origin.position.x, link->inertial->origin.position.y, link->inertial->origin.position.z;

                    m_inertialParameters.template segment<3>(1) = m_inertialParameters(0) * t_translationCOM;

                    Eigen::Matrix<T, 3, 3> t_rotInertiaWRTOrigin;
                    t_rotInertiaWRTOrigin(0, 0) = link->inertial->ixx;
                    t_rotInertiaWRTOrigin(1, 1) = link->inertial->iyy;
                    t_rotInertiaWRTOrigin(2, 2) = link->inertial->izz;
                    t_rotInertiaWRTOrigin(0, 1) = t_rotInertiaWRTOrigin(1, 0) = link->inertial->ixy;
                    t_rotInertiaWRTOrigin(0, 2) = t_rotInertiaWRTOrigin(2, 0) = link->inertial->ixz;
                    t_rotInertiaWRTOrigin(1, 2) = t_rotInertiaWRTOrigin(2, 1) = link->inertial->iyz;

                    Eigen::Matrix<T, 3, 3> t_rotInertiaWRTCOM = t_rotationCOM * t_rotInertiaWRTOrigin;

                    Eigen::Matrix<T, 3, 3> t_translationCOMSkew;
                    skewMatrix(t_translationCOM, t_translationCOMSkew);

                    t_rotInertiaWRTCOM = t_rotInertiaWRTCOM + (t_translationCOMSkew * t_translationCOMSkew.transpose() * m_inertialParameters(0));

                    m_inertialParameters(4) = t_rotInertiaWRTCOM(0, 0);//xx
                    m_inertialParameters(5) = t_rotInertiaWRTCOM(1, 1);//yy
                    m_inertialParameters(6) = t_rotInertiaWRTCOM(2, 2);//zz
                    m_inertialParameters(7) = t_rotInertiaWRTCOM(0, 1);//xy
                    m_inertialParameters(8) = t_rotInertiaWRTCOM(0, 2);//xz
                    m_inertialParameters(9) = t_rotInertiaWRTCOM(1, 2);//yz
                    calculateSpatialInertia();
                }

                if (link->collision) {
                    m_hasCollision = true;
                    //If made of multiple collision primitives
                    // std::cout << "collision meshes: " << link->collision_array.size() << std::endl;
                    if (link->collision_array.size() > 1) {
                        m_collisionModel_sp = std::shared_ptr<fcl::BVHModel<fcl::OBB<T> > >(new fcl::BVHModel<fcl::OBB<T> >());
                        for (size_t i = 0; i < link->collision_array.size(); i++) {
                            mt_collisionOffset.setIdentity();
                            mt_collisionOffset.translation() = Eigen::Vector3f(link->collision_array[i]->origin.position.x, link->collision_array[i]->origin.position.y, link->collision_array[i]->origin.position.z).cast<T>();
                            Eigen::Quaterniond quat;
                            link->collision_array[i]->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                            mt_collisionOffset.linear() = quat.cast<T>().matrix();

                            std::shared_ptr<urdf::Mesh> t_mesh_sp = std::dynamic_pointer_cast<urdf::Mesh>(link->collision_array[i]->geometry);
                            if (t_mesh_sp) {
                                Eigen::Matrix<T, 3, 1> t_scale(t_mesh_sp->scale.x, t_mesh_sp->scale.y, t_mesh_sp->scale.z);
                                m_collisionFiles.push_back(t_mesh_sp->filename);
                                createMesh<fcl::OBB<T> >(m_collisionModel_sp, t_scale, t_mesh_sp->filename, mt_collisionOffset);
                            } else {
                                std::shared_ptr<urdf::Cylinder> cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(link->collision_array[i]->geometry);
                                if (cylinder) {
                                    createCylinder<fcl::OBB<T> >(m_collisionModel_sp, (T)cylinder->radius, (T)cylinder->length, mt_collisionOffset);
                                    m_collisionFiles.push_back("CYLINDER");
                                } else {
                                    std::shared_ptr<urdf::Box> box = std::dynamic_pointer_cast<urdf::Box>(link->collision_array[i]->geometry);
                                    if (box) {
                                        Eigen::Matrix<T, 3, 1> scale;
                                        scale << (T)box->dim.x, (T)box->dim.y, (T)box->dim.z;
                                        createBox<fcl::OBB<T> >(m_collisionModel_sp, scale, mt_collisionOffset);
                                        m_collisionFiles.push_back("BOX");
                                    } else {
                                        std::shared_ptr<urdf::Sphere> sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->collision_array[i]->geometry);
                                        if (sphere) {
                                            createSphere<fcl::OBB<T> >(m_collisionModel_sp, (T)sphere->radius, mt_collisionOffset);
                                            m_collisionFiles.push_back("SPHERE");
                                        }
                                    }
                                }
                            }
                        }
                        m_collisionModel_sp->endModel();
                        m_collisionModel_sp->computeLocalAABB();
                    } else {
                        //made of a single collision object
                        m_collisionModel_sp = std::shared_ptr<fcl::BVHModel<fcl::OBB<T> > >(new fcl::BVHModel<fcl::OBB<T> >());
                        mt_collisionOffset.setIdentity();
                        mt_collisionOffset.translation() = Eigen::Vector3f(link->collision->origin.position.x, link->collision->origin.position.y, link->collision->origin.position.z).cast<T>();
                        Eigen::Quaterniond quat;
                        link->collision->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                        mt_collisionOffset.linear() = quat.cast<T>().matrix();

                        std::shared_ptr<urdf::Mesh> t_mesh_sp = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
                        if (t_mesh_sp) {
                            Eigen::Matrix<T, 3, 1> t_scale(t_mesh_sp->scale.x, t_mesh_sp->scale.y, t_mesh_sp->scale.z);
                            m_collisionFiles.push_back(t_mesh_sp->filename);
                            createMesh<fcl::OBB<T> >(m_collisionModel_sp, t_scale, t_mesh_sp->filename, mt_collisionOffset);
                        } else {
                            std::shared_ptr<urdf::Cylinder> cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
                            if (cylinder) {
                                createCylinder<fcl::OBB<T> >(m_collisionModel_sp, (T)cylinder->radius, (T)cylinder->length, mt_collisionOffset);
                                m_collisionFiles.push_back("CYLINDER");
                            } else {
                                std::shared_ptr<urdf::Box> box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
                                if (box) {
                                    Eigen::Matrix<T, 3, 1> scale;
                                    scale << (T)box->dim.x, (T)box->dim.y, (T)box->dim.z;
                                    createBox<fcl::OBB<T> >(m_collisionModel_sp, scale, mt_collisionOffset);
                                    m_collisionFiles.push_back("BOX");
                                } else {
                                    std::shared_ptr<urdf::Sphere> sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);
                                    if (sphere) {
                                        createSphere<fcl::OBB<T> >(m_collisionModel_sp, (T)sphere->radius, mt_collisionOffset);
                                        m_collisionFiles.push_back("SPHERE");
                                    }
                                }
                            }
                        }

                        m_collisionModel_sp->endModel();
                        m_collisionModel_sp->computeLocalAABB();
                    }
                    m_collisionObject_sp.reset(new fcl::CollisionObject<T>(m_collisionModel_sp));
                }
            }

            Link(urdf::LinkConstSharedPtr link) : Link(link, false, nullptr) {};

            ~Link() {}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            std::vector<std::shared_ptr<Joint<T> > > loadChildJoints(urdf::LinkConstSharedPtr link) {
                if (!m_isRoot) {
                    m_parentJoint_sp = m_parentLink_sp->getJointGoingToChild(m_name);

                    if (!m_parentJoint_sp->isFixed()) {
                        m_inertialParameters[10] = T(m_parentJoint_sp->getViscousFriction());
                        m_inertialParameters[11] = T(m_parentJoint_sp->getStaticFriction());
                    }

                    if (m_parentJoint_sp->isFixed() && !m_parentLink_sp->isRoot()) {
                        std::shared_ptr<Link<T> > t_link = m_parentLink_sp;
                        while (t_link->getParentJoint()->isFixed()) {
                            t_link = t_link->getParentLink();
                        }

                        AdjointSE3<T> t_fixedTransform;
                        t_fixedTransform = m_parentJoint_sp->getOriginTransform();
                        if (m_hasInertial) {
                            t_fixedTransform.applyXIX(t_link->getInertialParams().template segment<10>(0), t_link->getInertialParams().template segment<10>(0));
                            t_link->setInertialParams(t_link->getInertialParams().template segment<10>(0) + m_inertialParameters.template segment<10>(0));
                        }
                    }
                }

                std::vector<std::shared_ptr<Joint<T> > > jointList;
                for (urdf::JointSharedPtr joint : link->child_joints) {
                    switch (joint->type) {
                    case urdf::Joint::REVOLUTE:
                        m_childJoints_sm[joint->child_link_name].reset(new RevoluteJoint<T>(joint, this->shared_from_this()));
                        jointList.push_back(m_childJoints_sm[joint->child_link_name]);
                        break;
                    case urdf::Joint::PRISMATIC:
                        throw std::runtime_error("PRISMATIC JOINT NOT IMPLEMENTED");
                        break;
                    case urdf::Joint::FIXED:
                        m_childJoints_sm[joint->child_link_name].reset(new FixedJoint<T>(joint, this->shared_from_this()));
                        jointList.push_back(m_childJoints_sm[joint->child_link_name]);
                        break;
                    default:
                        break;
                    }
                }

                return jointList;
            }

            /***************** END INITIALIZATION HANDLING *****************/

            Eigen::Matrix<T, 12, 1> &getInertialParams() {
                return m_inertialParameters;
            }

            template <typename Derived> void setInertialParams(const Eigen::MatrixBase<Derived> &params) {
                m_inertialParameters.template segment(0, params.size()) = params;
                calculateSpatialInertia();
            }

            std::shared_ptr<Joint<T> > getJointGoingToChild(const std::string &child_link) {
                return m_childJoints_sm[child_link];
            }
            std::shared_ptr<Joint<T> > getParentJoint() {
                return m_parentJoint_sp;
            }
            std::shared_ptr<Link<T> > getParentLink() {
                return m_parentLink_sp;
            }

            std::string const &getName() const {
                return m_name;
            }

            bool operator ==(const Link<T> &rhs) {
                return this->m_name == rhs.getName();
            }
            bool operator ==(const std::string &rhs) {
                return this->m_name == rhs;
            }

            template <typename Derived> void updateParams(const Eigen::MatrixBase<Derived> &paramDot) {
                m_inertialParameters.template segment(0, paramDot.size()) += paramDot;
                calculateSpatialInertia();
            }

            bool isRoot() {
                return m_isRoot;
            }

            const SpatialInertia<T> &getSpatialInertia() {
                return m_spatialInertia;
            }

            /***************** COLLISION HANDLING *****************/
            /**
             * @brief Get the Collision Mesh File object
             *
             * @return std::string Filename if it exists otherwise returns an empty string
             */
            std::string getCollisionMeshFile() {
                if (m_hasCollision) {
                    return m_collisionFiles[0];
                } else {
                    return "";
                }
            }

            void updateCollision(const AdjointSE3<T> &at) {
                if (m_hasCollision) {
                    updateCollision_us(at);
                }
            }
            void updateCollision_us(const AdjointSE3<T> &at) {
                m_collisionObject_sp->setTranslation(at.getP().template cast<T>());
                m_collisionObject_sp->setRotation(at.getR().template cast<T>());
            }

            bool hasCollision() {
                return m_hasCollision;
            }

            std::shared_ptr<fcl::CollisionObject<T> > getCollisionObjectPtr() {
                return m_collisionObject_sp;
            }

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getCollisionVectors() {
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> output;
                int vertices = m_collisionModel_sp->num_vertices;
                output.resize(vertices, 3);
                for (size_t i = 0; i < vertices; i++) {
                    output.row(i) = m_collisionModel_sp->vertices[i].transpose();
                }
                return output;
            }

            std::vector<std::array<int, 3> > getCollisionTriangles() {
                std::vector<std::array<int, 3> > output;
                int vertices = m_collisionModel_sp->num_tris;
                output.resize(vertices);
                for (size_t i = 0; i < vertices; i++) {
                    output[i][0] = m_collisionModel_sp->tri_indices[i][0];
                    output[i][1] = m_collisionModel_sp->tri_indices[i][1];
                    output[i][2] = m_collisionModel_sp->tri_indices[i][2];
                }
                return output;
            }

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T> > > getCollisionModel() {
                return m_collisionModel_sp;
            }
            /***************** END COLLISION HANDLING *****************/

            urdf::LinkConstSharedPtr getUrdfLink() {
                return urdfModel;
            }
        };
    }
}

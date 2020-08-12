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
        class Link: public std::enable_shared_from_this<Link<T>> {
             private:
#if !ARDL_EXTERNAL_DATA
            bool m_isRoot, m_hasInertial, m_hasCollision;
            std::string m_name;

            Eigen::Matrix<T, 12, 1> m_inertialParameters;

            JointVariantSharedPtr<T> m_parentJoint_sp;
            std::shared_ptr<Link<T>> m_parentLink_sp;

            std::map<std::string, JointVariantSharedPtr<T>> m_childJoints_sm;

            fcl::Transform3<T> mt_collisionOffset;

            const Eigen::Matrix<T, 3, 3> mt_identity3x3= Eigen::Matrix<T, 3, 3>::Identity();

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>> m_collisionModel_sp, m_collisionModelOptimal_sp;
            std::shared_ptr<fcl::CollisionObject<T>> m_collisionObject_sp;
            std::vector<std::string> m_collisionFiles;

            SpatialInertia<T> m_sI;

            urdf::LinkConstSharedPtr urdfModel;
#else
            bool &m_isRoot, &m_hasInertial, &m_hasCollision;
            std::string &m_name;

            Eigen::Matrix<T, 12, 1> &m_inertialParameters;

            int &m_parentJoint_index;
            int &m_parentLink_index;

            std::map<std::string, urdf::JointSharedPtr<T>> &m_childJoints_sm;

            fcl::Transform3<T> &mt_collisionOffset;

            const Eigen::Matrix<T, 3, 3> &mt_identity3x3;

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>> &m_collisionModel_sp, &m_collisionModelOptimal_sp;
            std::shared_ptr<fcl::CollisionObject<T>> &m_collisionObject_sp;
            std::vector<std::string> &m_collisionFiles;

            SpatialInertia<T> &m_sI;

            urdf::LinkConstSharedPtr &urdfModel;

            ARDL::Data<T> &m_data;
#endif

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
#if !ARDL_EXTERNAL_DATA
            Link(const Link<T> &copy)
            // #else
            //             Link(const Link<T> &copy, ARDL::Data<T> &data, size_t i)
            //                 : m_data(data), m_isRoot(data.isRoot[i]), m_hasInertial(data.hasInertial[i]),
            //                   m_hasCollision(data.hasCollision[i]), m_name(data.name[i]),
            //                   m_inertialParameters(data.inertialParameters[i]),
            //                   m_parentJoint_sp(data.parentJoint_sp[i]), m_parentLink_sp(data.parentLink_sp[i]),
            //                   m_childJoints_sm(data.childJoints_sm[i]),
            //                   mt_collisionOffset(data.mt_collisionOffset[i]), mt_identity3x3(data.mt_identity3x3),
            //                   m_collisionModel_sp(data.collisionModel_sp[i]),
            //                   m_collisionModelOptimal_sp(data.collisionModelOptimal_sp[i]),
            //                   m_collisionObject_sp(data.collisionObject_sp[i]),
            //                   m_collisionFiles(data.collisionFiles[i]), m_sI(data.sI[i]),
            //                   urdfModel(data.urdfModel[i])
            // #endif
            {
                m_isRoot= copy.m_isRoot;
                m_hasInertial= copy.m_hasInertial;
                m_hasCollision= copy.m_hasCollision;

                m_name= copy.m_name;
                if(m_hasInertial) {
                    m_inertialParameters= copy.m_inertialParameters;
                    m_sI= copy.m_sI;
                }
    #if !ARDL_EXTERNAL_DATA
                m_parentLink_sp= nullptr;
    #else
                m_parentLink_index= -1;
                m_parentJoint_index= -1;
    #endif
                if(m_hasCollision) {
                    mt_collisionOffset= copy.mt_collisionOffset;
                    m_collisionModel_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                        new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModel_sp)));
                    m_collisionModelOptimal_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                        new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModelOptimal_sp)));
                    m_collisionObject_sp= std::shared_ptr<fcl::CollisionObject<T>>(
                        new fcl::CollisionObject<T>(*(copy.m_collisionObject_sp)));
                    m_collisionFiles= copy.m_collisionFiles;
                }
                this->urdfModel= copy.urdfModel;
            }
#endif
#if !ARDL_EXTERNAL_DATA
            Link(const Link<T> &copy, JointVariantSharedPtr<T> parentJoint, std::shared_ptr<Link<T>> parentLink)
#else
            Link(const Link<T> &copy, ARDL::Data<T> &data, size_t i)
                : m_data(data), m_isRoot(data.isRoot[i]), m_hasInertial(data.hasInertial[i]),
                  m_hasCollision(data.hasCollision[i]), m_name(data.name[i]),
                  m_inertialParameters(data.inertialParameters[i]), m_parentJoint_index(data.parentJoint_index[i]),
                  m_parentLink_index(data.parentLink_index[i]), m_childJoints_sm(data.childJoints_sm[i]),
                  mt_collisionOffset(data.mt_collisionOffset[i]), mt_identity3x3(data.mt_identity3x3),
                  m_collisionModel_sp(data.collisionModel_sp[i]),
                  m_collisionModelOptimal_sp(data.collisionModelOptimal_sp[i]),
                  m_collisionObject_sp(data.collisionObject_sp[i]), m_collisionFiles(data.collisionFiles[i]),
                  m_sI(data.sI[i]), urdfModel(data.urdfModel[i])
#endif
            {
                m_isRoot= copy.m_isRoot;
                m_hasInertial= copy.m_hasInertial;
                m_hasCollision= copy.m_hasCollision;

                m_name= copy.m_name;
                if(m_hasInertial) {
                    m_inertialParameters= copy.m_inertialParameters;
                    m_sI= copy.m_sI;
                }

#if !ARDL_EXTERNAL_DATA
                m_parentJoint_sp= parentJoint;
                m_parentLink_sp= parentLink;
#else
                m_parentLink_index= copy.m_parentLink_index;
                m_parentJoint_index= copy.m_parentJoint_index;
#endif
                if(m_hasCollision) {
                    mt_collisionOffset= copy.mt_collisionOffset;
                    m_collisionModel_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                        new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModel_sp)));
                    m_collisionModelOptimal_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                        new fcl::BVHModel<fcl::OBB<T>>(*(copy.m_collisionModelOptimal_sp)));
                    m_collisionObject_sp= std::shared_ptr<fcl::CollisionObject<T>>(
                        new fcl::CollisionObject<T>(*(copy.m_collisionObject_sp)));
                    m_collisionFiles= copy.m_collisionFiles;
                }
                this->urdfModel= copy.urdfModel;
            }
            // Link(const Link<T> &copy) = delete;

            /***************** INITIALIZATION HANDLING *****************/

#if !ARDL_EXTERNAL_DATA
            Link(urdf::LinkConstSharedPtr link, bool root, std::shared_ptr<Link<T>> parent_sp)
                : urdfModel(link)
#else
            Link(urdf::LinkConstSharedPtr link, bool root, int parentLink, int parentJoint, ARDL::Data<T> &data,
                 size_t i)
                : urdfModel(link), m_data(data), m_isRoot(data.isRoot[i]), m_hasInertial(data.hasInertial[i]),
                  m_hasCollision(data.hasCollision[i]), m_name(data.name[i]),
                  m_inertialParameters(data.inertialParameters[i]), m_parentJoint_index(data.parentJoint_index[i]),
                  m_parentLink_index(data.parentLink_index[i]), m_childJoints_sm(data.childJoints_sm[i]),
                  mt_collisionOffset(data.mt_collisionOffset[i]), mt_identity3x3(data.mt_identity3x3),
                  m_collisionModel_sp(data.collisionModel_sp[i]),
                  m_collisionModelOptimal_sp(data.collisionModelOptimal_sp[i]),
                  m_collisionObject_sp(data.collisionObject_sp[i]), m_collisionFiles(data.collisionFiles[i]),
                  m_sI(data.sI[i]), urdfModel(data.urdfModel[i])
#endif
            {
                m_isRoot= root;
                m_name= link->name;
#if !ARDL_EXTERNAL_DATA
                m_parentLink_sp= parent_sp;
#else
                m_parentLink_index= parentLink;
                m_parentJoint_index= parentJoint;
#endif
                if(link->inertial) {
                    m_hasInertial= true;
                    m_inertialParameters.setZero();
                    m_inertialParameters(0)= link->inertial->mass;

                    Eigen::Quaterniond t_quaternion;
                    link->inertial->origin.rotation.getQuaternion(t_quaternion.x(), t_quaternion.y(), t_quaternion.z(),
                                                                  t_quaternion.w());

                    Eigen::Matrix<T, 3, 3> t_rotationCOM= t_quaternion.toRotationMatrix().cast<T>();

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

                    if(!m_isRoot && link->getParent()->child_joints[0]->child_link_name == m_name &&
                       parent_sp->m_hasCollision &&
                       ARDL_visit_ptr(parent_sp->m_childJoints_sm[link->getParent()->child_joints[0]->child_link_name],
                                      isFixed())) {
                        m_collisionModelOptimal_sp=
                            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(new fcl::BVHModel<fcl::OBB<T>>());
                        std::vector<Matrix<T, 3, 1>> ps;
                        std::vector<fcl::Triangle> ts;

                        for(size_t i= 0; i < m_collisionModel_sp->num_vertices; i++) {
                            ps.push_back((m_collisionModel_sp->vertices)[i]);
                        }
                        for(size_t i= 0; i < m_collisionModel_sp->num_tris; i++) {
                            ts.push_back(m_collisionModel_sp->tri_indices[i]);
                        }

#if !ARDL_EXTERNAL_DATA
                        AdjointSE3 transform(
                            ARDL_visit_ptr(m_parentLink_sp->getJointGoingToChild(m_name), getOriginTransform()));
#else
                        AdjointSE3 transform(
                            ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], getOriginTransform()));
#endif
                        transform.inverse();
                        mt_collisionOffset.translation()= transform.getP();
                        mt_collisionOffset.linear()= transform.getR();

#if !ARDL_EXTERNAL_DATA
                        for(size_t i= 0; i < m_parentLink_sp->m_collisionModel_sp->num_vertices; i++) {
                            ps.push_back(mt_collisionOffset * (m_parentLink_sp->m_collisionModel_sp->vertices)[i]);
                        }
                        for(size_t i= 0; i < m_parentLink_sp->m_collisionModel_sp->num_tris; i++) {
                            ts.push_back(m_parentLink_sp->m_collisionModel_sp->tri_indices[i]);
#else
                        for(size_t i= 0; i < m_data.links_vsp[m_parentLink_index]->m_collisionModel_sp->num_vertices;
                            i++) {
                            ps.push_back(mt_collisionOffset *
                                         (m_data.links_vsp[m_parentLink_index]->m_collisionModel_sp->vertices)[i]);
                        }
                        for(size_t i= 0; i < m_data.links_vsp[m_parentLink_index]->m_collisionModel_sp->num_tris; i++) {
                            ts.push_back(m_data.links_vsp[m_parentLink_index]->m_collisionModel_sp->tri_indices[i]);
#endif
                            ts.back()[0]+= m_collisionModel_sp->num_vertices;
                            ts.back()[1]+= m_collisionModel_sp->num_vertices;
                            ts.back()[2]+= m_collisionModel_sp->num_vertices;
                        }

                        m_collisionModelOptimal_sp->beginModel();
                        m_collisionModelOptimal_sp->addSubModel(ps, ts);
                        m_collisionModelOptimal_sp->endModel();
                        m_collisionModelOptimal_sp->computeLocalAABB();
                    } else if(m_hasCollision) {
                        m_collisionModelOptimal_sp= std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>>(
                            new fcl::BVHModel<fcl::OBB<T>>(*(m_collisionModel_sp)));
                    }
                }
                #if ARDL_EXTERNAL_DATA
                if(!m_isRoot) {
                    if(!ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], isFixed())) {
                        m_inertialParameters[10]= T(ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], getViscousFriction()));
                        m_inertialParameters[11]= T(ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], getStaticFriction()));
                    }
                    
                    if(ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], isFixed()) && !m_data.links_vsp[m_parentLink_index]->isRoot()) {
                        AdjointSE3<T> t_fixedTransform= ARDL_visit_ptr(m_data.joints_vsp[m_parentJointIndex], getOriginTransform());
                        int parentJIndex = m_data.links_vsp[m_parentLink_index].m_parentJoint_index;
                        int parentLIndex = m_parentLink_index;
                        while(ARDL_visit_ptr(m_data.joints_vsp[t_link.m_parentJointIndex], isFixed())) {
                            t_fixedTransform.apply(ARDL_visit_ptr(m_data.joints_vsp[parentJIndex], getOriginTransform()));

                            parentLIndex= m_data.links_vsp[parentLIndex].m_parentLink_index;
                            parentJIndex=m_data.links_vsp[parentLIndex].m_parentJoint_index;
                        }
                        if(m_hasInertial) {
                            m_sI.applyInverseXIX(t_fixedTransform);
                            m_data.sI[parentLIndex]+= m_sI;
                        }
                    }
                }
                #endif
            }
#if !ARDL_EXTERNAL_DATA
            Link(urdf::LinkConstSharedPtr link): Link(link, false, nullptr){};
#else
            Link(urdf::LinkConstSharedPtr link, ARDL::Data<T> &data, size_t i): Link(link, false, -1, -1, data, i){};
#endif

            ~Link() {}

            void loadCollision(const urdf::CollisionSharedPtr &collision) {
                mt_collisionOffset.setIdentity();
                mt_collisionOffset.translation()=
                    Eigen::Vector3f(collision->origin.position.x, collision->origin.position.y,
                                    collision->origin.position.z)
                        .cast<T>();
                Eigen::Quaterniond quat;
                collision->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
                mt_collisionOffset.linear()= quat.cast<T>().matrix();

                std::shared_ptr<urdf::Mesh> t_mesh_sp= std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
                if(t_mesh_sp) {
                    Eigen::Matrix<T, 3, 1> t_scale(t_mesh_sp->scale.x, t_mesh_sp->scale.y, t_mesh_sp->scale.z);
                    m_collisionFiles.push_back(t_mesh_sp->filename);
                    createMesh<fcl::OBB<T>>(m_collisionModel_sp, t_scale, t_mesh_sp->filename, mt_collisionOffset);
                } else {
                    std::shared_ptr<urdf::Cylinder> cylinder=
                        std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
                    if(cylinder) {
                        createCylinder<fcl::OBB<T>>(m_collisionModel_sp, (T) cylinder->radius, (T) cylinder->length,
                                                    mt_collisionOffset);
                        m_collisionFiles.push_back("CYLINDER");
                    } else {
                        std::shared_ptr<urdf::Box> box= std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
                        if(box) {
                            Eigen::Matrix<T, 3, 1> scale;
                            scale << (T) box->dim.x, (T) box->dim.y, (T) box->dim.z;
                            createBox<fcl::OBB<T>>(m_collisionModel_sp, scale, mt_collisionOffset);
                            m_collisionFiles.push_back("BOX");
                        } else {
                            std::shared_ptr<urdf::Sphere> sphere=
                                std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                            if(sphere) {
                                createSphere<fcl::OBB<T>>(m_collisionModel_sp, (T) sphere->radius, mt_collisionOffset);
                                m_collisionFiles.push_back("SPHERE");
                            }
                        }
                    }
                }
            }

            void loadChildJoints(urdf::LinkConstSharedPtr link) {
#if !ARDL_EXTERNAL_DATA
                if(!m_isRoot) {
                    // maybe redo this for external data
                    m_parentJoint_sp= m_parentLink_sp->getJointGoingToChild(
                        m_name); // std::allocate_shared<JointVariant<T> >(Eigen::aligned_allocator<JointVariant<T> >(),
                                 // std::move(m_parentLink_sp->getJointGoingToChild(m_name)));
                    if(!ARDL_visit_ptr(m_parentJoint_sp, isFixed())) {
                        m_inertialParameters[10]= T(ARDL_visit_ptr(m_parentJoint_sp, getViscousFriction()));
                        m_inertialParameters[11]= T(ARDL_visit_ptr(m_parentJoint_sp, getStaticFriction()));
                    }

                    if(ARDL_visit_ptr(m_parentJoint_sp, isFixed()) && !m_parentLink_sp->isRoot()) {
                        std::shared_ptr<Link<T>> t_link= m_parentLink_sp;
                        AdjointSE3<T> t_fixedTransform= ARDL_visit_ptr(m_parentJoint_sp, getOriginTransform());

                        while(ARDL_visit_ptr(t_link->getParentJoint(), isFixed())) {
                            t_fixedTransform.apply(ARDL_visit_ptr(t_link->m_parentJoint_sp, getOriginTransform()));
                            t_link= t_link->getParentLink();
                        }
                        if(m_hasInertial) {
                            m_sI.applyInverseXIX(t_fixedTransform);
                            t_link->getSI()+= m_sI;
                        }
                    }
                }
#endif
                for(urdf::JointSharedPtr joint: link->child_joints) {
                    switch(joint->type) {
                    case urdf::Joint::REVOLUTE:
                        m_childJoints_sm[joint->child_link_name]=
#if !ARDL_EXTERNAL_DATA
                            std::shared_ptr<RevoluteJoint<T>>(new RevoluteJoint<T>(joint, this->shared_from_this()));
#else
                            joint;
#endif
                        break;
                    case urdf::Joint::PRISMATIC: throw std::runtime_error("PRISMATIC JOINT NOT IMPLEMENTED"); break;
                    case urdf::Joint::FIXED:
                        m_childJoints_sm[joint->child_link_name]=
#if !ARDL_EXTERNAL_DATA
                            std::shared_ptr<FixedJoint<T>>(new FixedJoint<T>(joint, this->shared_from_this()));
#else
                            joint;
#endif
                        break;
                    default: break;
                    }
                }
            }

            /***************** END INITIALIZATION HANDLING *****************/

            SpatialInertia<T> &getSI() { return m_sI; }

            template<typename Derived>
            void setInertialParams(const Eigen::MatrixBase<Derived> &params) {
                m_sI= params;
            }
#if !ARDL_EXTERNAL_DATA
            JointVariantSharedPtr<T>
#else
            urdf::JointSharedPtr
#endif
                &getJointGoingToChild(const std::string &child_link) {
                return m_childJoints_sm[child_link];
            }
            JointVariantSharedPtr<T> &getParentJoint() { return m_parentJoint_sp; }
            std::shared_ptr<Link<T>> getParentLink() { return m_parentLink_sp; }

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

            void updateCollision(const AdjointSE3<T> &at) {
                if(m_hasCollision) { updateCollision_us(at); }
            }
            void updateCollision_us(const AdjointSE3<T> &at) {
                m_collisionObject_sp->setTranslation(at.getP().template cast<T>());
                m_collisionObject_sp->setRotation(at.getR().template cast<T>());
            }

            bool hasCollision() { return m_hasCollision; }

            std::shared_ptr<fcl::CollisionObject<T>> getCollisionObjectPtr() { return m_collisionObject_sp; }

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getCollisionVertices() {
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> output;
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

            std::shared_ptr<fcl::BVHModel<fcl::OBB<T>>> getCollisionModel() { return m_collisionModel_sp; }
            /***************** END COLLISION HANDLING *****************/

            urdf::LinkConstSharedPtr getUrdfLink() { return urdfModel; }
        };
    } // namespace Model
} // namespace ARDL

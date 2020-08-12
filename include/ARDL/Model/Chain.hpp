#pragma once

#include <string>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#if ARDL_EXTERNAL_DATA
    #include "ARDL/Data/Data.hpp"
#else
    #include "ARDL/Model/Joints/Joints.hpp"
    #include "ARDL/Model/Joint.hpp"
    #include "ARDL/Model/Link.hpp"
#endif
#include "ARDL/Util/Search.hpp"
#include "ARDL/Util/Overloaded.hpp"

namespace ARDL {
    namespace Model {
        /**
         * @brief Model of a single chain
         *
         * @tparam T
         */
        template<typename T>
        class Chain {
             private:
#if !ARDL_EXTERNAL_DATA
            // List of links (all/movable)
            aligned_vector<Link<T>> m_links_vsp;
            Link<T>* m_ee_link;
            aligned_vector<Link<T> *> m_movable_links_vsp;
            // List of movable joints
            aligned_vector<JointVariant<T>> m_joints_all_vsp;
            aligned_vector<JointVariant<T> *> m_joints_vsp;
            // Number of movable joints
            size_t m_jointNum;
            // Root name of chain and tip name of chain
            std::string m_rootName, m_tipName;

            // current position and velocity states of the chain
            Eigen::Matrix<T, Eigen::Dynamic, 1> q, qd;
#else
            // List of links (all/movable)
            aligned_vector<Link<T>> &m_links_vsp;
            aligned_vector<Link<T> *> &m_movable_links_vsp;
            // List of movable joints
            aligned_vector<JointVariant<T>> &m_joints_all_vsp;
            aligned_vector<JointVariant<T> *> &m_joints_vsp;
            // Number of movable joints
            size_t &m_jointNum;
            // Root name of chain and tip name of chain
            std::string &m_rootName, &m_tipName;

            // current position and velocity states of the chain
            Eigen::Matrix<T, Eigen::Dynamic, 1> &q, &qd;

            ARDL::Data<T> &m_data;
#endif

            /**
             * @brief Initialize the chain
             *
             * @param root_name Root link name
             * @param tip_name Tip link name
             * @param model The urdf model of the robot in question
             */
            void init(std::string root_name, std::string tip_name, urdf::ModelInterfaceSharedPtr model) {
                urdf::LinkConstSharedPtr m_rootLink= model->getLink(root_name);

                // Search to build chain between root to tip (designed for multibranching also for failure checking)
                std::stack<urdf::LinkConstSharedPtr> t_urdfChain= Util::Search::searchBreadth(
                    m_rootLink,
                    [](urdf::LinkConstSharedPtr l, std::queue<urdf::LinkConstSharedPtr> &q,
                       std::map<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr> &m) {
                        for(urdf::LinkConstSharedPtr childL: l->child_links) {
                            if(m.find(childL) != m.end()) { continue; }
                            q.push(childL);
                            m.insert(std::pair<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr>(childL, l));
                        }

                        return;
                    },
                    tip_name);

                m_rootName= root_name;
                m_tipName= tip_name;

                if(t_urdfChain.size() == 0) {
                    throw std::runtime_error("URDF is of size 0");
                } else {
                    m_jointNum= 0;

#if !ARDL_EXTERNAL_DATA
                    m_links_vsp.reserve(t_urdfChain.size());
                    m_movable_links_vsp.reserve(t_urdfChain.size());
                    m_joints_all_vsp.reserve(t_urdfChain.size());
                    m_joints_vsp.reserve(t_urdfChain.size());

                    this->m_links_vsp.emplace_back(t_urdfChain.top(), nullptr, nullptr);
                    t_urdfChain.pop();
                    this->m_links_vsp.back().loadChildJoint(t_urdfChain.top()->name, m_joints_all_vsp);

                    this->m_movable_links_vsp.push_back(&(this->m_links_vsp.back()));
                    while(!t_urdfChain.empty()) {
                        this->m_links_vsp.emplace_back(t_urdfChain.top(), &(this->m_links_vsp.back()),
                                                       &(this->m_joints_all_vsp.back()));

                        t_urdfChain.pop();
                        if(!t_urdfChain.empty()) {
                            this->m_links_vsp.back().loadChildJoint(t_urdfChain.top()->name, m_joints_all_vsp);
                            // if(!ARDL_visit(m_joints_all_vsp.back(), isFixed())) {
                            //                             if(!ARDL_visit(this->m_links_vsp.back().getParentJoint(),isFixed())){
                            //                             m_jointNum++;
                            //                             this->m_movable_links_vsp.push_back(&(this->m_links_vsp.back()));
                            //                             // this->m_joints_vsp.push_back(&(m_joints_all_vsp.back()));
                            // this->m_joints_vsp.push_back(&(this->m_links_vsp.back().getParentJoint()));
                            //                             // this->m_links_vsp.back().getParentJoint()
                            //                         }
                        }else{
                            m_ee_link = &this->m_links_vsp.back();
                        }
                        // else{
                        //     this->m_movable_links_vsp.push_back(&(this->m_links_vsp.back()));
                        // }
                        if(!ARDL_visit(this->m_links_vsp.back().getParentJoint(), isFixed())) {
                            m_jointNum++;
                            this->m_movable_links_vsp.push_back(&(this->m_links_vsp.back()));
                            this->m_joints_vsp.push_back(&(this->m_links_vsp.back().getParentJoint()));
                        }
                    }
#else
                    m_data.addLink();
                    this->m_links_vsp.emplace_back(t_urdfChain.top(), nullptr, nullptr, m_data, 0);
                    t_urdfChain.pop();
                    this->m_links_vsp.back().loadChildJoint(t_urdfChain.top()->name, m_data, 0);
                    while(!t_urdfChain.empty()) {
                        m_data.addLink();
                        this->m_links_vsp.emplace_back(t_urdfChain.top(), &(this->m_links_vsp.back()),
                                                       &(this->m_joints_all_vsp.back()), m_data,
                                                       this->m_links_vsp.size());

                        t_urdfChain.pop();
                        if(!t_urdfChain.empty()) {
                            this->m_links_vsp.back().loadChildJoint(t_urdfChain.top()->name, m_data,
                                                                    this->m_joints_all_vsp.size());
                        }
                        if(!ARDL_visit(this->m_links_vsp.back().getParentJoint(), isFixed())) {
                            m_jointNum++;
                            this->m_movable_links_vsp.push_back(&(this->m_links_vsp.back()));
                            this->m_joints_vsp.push_back(&(this->m_links_vsp.back().getParentJoint()));
                        }
                    }
#endif
                    q.resize(m_jointNum);
                    qd.resize(m_jointNum);
                }
            }

             public:
#if !ARDL_EXTERNAL_DATA
            Chain(const Chain<T> &copy)
#else
            Chain(const Chain<T> &copy, ARDL::Data<T> &data)
                : m_links_vsp(data.links), m_movable_links_vsp(data.movable_links), m_joints_all_vsp(data.joints_all),
                  m_joints_vsp(data.joints), m_jointNum(data.jointNum), m_rootName(data.rootName),
                  m_tipName(data.tipName), q(data.q), qd(data.qd), m_data(data)
#endif
            {
                m_jointNum= copy.m_jointNum;
                m_rootName= copy.m_rootName;
                m_tipName= copy.m_tipName;
                q= copy.q;
                qd= copy.qd;

#if !ARDL_EXTERNAL_DATA
                m_links_vsp.emplace_back(copy.m_links_vsp[0], nullptr, nullptr);
                for(size_t i= 1; i < copy.m_links_vsp.size(); i++) {
                    std::ptrdiff_t parentLIndex= &(copy.m_links_vsp[i].getParentLink()) - copy.m_links_vsp.data();
                    std::ptrdiff_t parentJIndex= &(copy.m_links_vsp[i].getParentJoint()) - copy.m_joints_all_vsp.data();
    #if ARDL_VARIANT == ON
        #if VISITORDER
                    ARDL_VISIT(
                        copy.m_joints_all_vsp[i - 1],
                        [&](RevoluteJoint<T> &joint) {
                            m_joints_all_vsp.emplace_back(RevoluteJoint<T>(joint, m_links_vsp[parentLIndex]));
                        },
                        [&](FixedJoint<T> &joint) {
                            m_joints_all_vsp.emplace_back(FixedJoint<T>(joint, m_links_vsp[parentLIndex]));
                        });
        #else
                    ARDL_VISIT(overloaded{[&](const RevoluteJoint<T> &joint) {
                                              m_joints_all_vsp.emplace_back(
                                                  RevoluteJoint<T>(joint, m_links_vsp[parentLIndex]));
                                          },
                                          [&](const FixedJoint<T> &joint) {
                                              m_joints_all_vsp.emplace_back(
                                                  FixedJoint<T>(joint, m_links_vsp[parentLIndex]));
                                          }},
                               copy.m_joints_all_vsp[i - 1]);
        #endif
                    m_links_vsp.emplace_back(copy.m_links_vsp[i], &(m_joints_all_vsp[parentJIndex]),
                                             &(m_links_vsp[parentLIndex]));

    #else
                    std::shared_ptr<RevoluteJoint<T>> joint_ptr=
                        std::dynamic_pointer_cast<RevoluteJoint<T>>(&(copy.m_joints_all_vsp[i - 1]));
                    if(joint_ptr) {
                        m_joints_all_vsp.push_back(RevoluteJoint<T>(*(joint_ptr), &(m_links_vsp[parentLIndex])));
                        m_joints_vsp.push_back(&(m_joints_all_vsp.back()));
                    } else {
                        std::shared_ptr<FixedJoint<T>> fixed_ptr=
                            std::dynamic_pointer_cast<FixedJoint<T>>(&(copy.m_joints_all_vsp[i - 1]));
                        if(fixed_ptr) {
                            m_joints_all_vsp.push_back(FixedJoint<T>(*(joint_ptr), m_links_vsp[parentLIndex]));
                        }
                    }
                    m_links_vsp.emplace_back(copy.m_links_vsp[0], &(m_joints_all_vsp.back()),
                                             &(m_links_vsp[parentLIndex]));
                    if(joint_ptr) { m_movable_links_vsp.push_back(&(m_links_vsp.back())); }
    #endif
                }
                for(size_t i= 0; i < copy.m_movable_links_vsp.size(); i++) {
                    std::ptrdiff_t parentLIndex= copy.m_movable_links_vsp[i] - copy.m_links_vsp.data();
                    m_movable_links_vsp.push_back(m_links_vsp.data() + parentLIndex);
                }
                for(size_t i= 0; i < copy.m_joints_vsp.size(); i++) {
                    std::ptrdiff_t parentJIndex= copy.m_joints_vsp[i] - copy.m_joints_all_vsp.data();
                    m_joints_vsp.push_back(m_joints_all_vsp.data() + parentJIndex);
                }
#else
                m_links_vsp.clear();
                m_joints_all_vsp.clear();
                m_links_vsp.emplace_back(copy.m_links_vsp[0], nullptr, nullptr, data, 0);
                for(size_t i= 1; i < copy.m_links_vsp.size(); i++) {
                    std::ptrdiff_t parentLIndex= &(copy.m_links_vsp[i].getParentLink()) - copy.m_links_vsp.data();
                    std::ptrdiff_t parentJIndex= &(copy.m_links_vsp[i].getParentJoint()) - copy.m_joints_all_vsp.data();
    #if VISITORDER
                    ARDL_VISIT(
                        copy.m_joints_all_vsp[i - 1],
                        [&](RevoluteJoint<T> &joint) {
                            m_joints_all_vsp.emplace_back(
                                RevoluteJoint<T>(joint, m_links_vsp[parentLIndex], data, i - 1));
                        },
                        [&](FixedJoint<T> &joint) {
                            m_joints_all_vsp.emplace_back(FixedJoint<T>(joint, m_links_vsp[parentLIndex], data, i - 1));
                        });
    #else
                    ARDL_VISIT(overloaded{[&](RevoluteJoint<T> &joint) {
                                              m_joints_all_vsp.emplace_back(
                                                  RevoluteJoint<T>(joint, m_links_vsp[parentLIndex], data, i - 1));
                                          },
                                          [&](FixedJoint<T> &joint) {
                                              m_joints_all_vsp.emplace_back(
                                                  FixedJoint<T>(joint, m_links_vsp[parentLIndex], data, i - 1));
                                          }},
                               copy.m_joints_all_vsp[i - 1]);
    #endif
                    m_links_vsp.emplace_back(copy.m_links_vsp[i], &(m_joints_all_vsp[parentJIndex]),
                                             &(m_links_vsp[parentLIndex]), data, i);
                }
                m_movable_links_vsp.clear();
                for(size_t i= 0; i < copy.m_movable_links_vsp.size(); i++) {
                    std::ptrdiff_t parentLIndex= copy.m_movable_links_vsp[i] - copy.m_links_vsp.data();
                    m_movable_links_vsp.push_back(m_links_vsp.data() + parentLIndex);
                }
                m_joints_vsp.clear();
                for(size_t i= 0; i < copy.m_joints_vsp.size(); i++) {
                    std::ptrdiff_t parentJIndex= copy.m_joints_vsp[i] - copy.m_joints_all_vsp.data();
                    m_joints_vsp.push_back(m_joints_all_vsp.data() + parentJIndex);
                }
#endif
            }

            /**
             * @brief Construct a new Chain object
             * Only works with single chain robots.
             *
             * @param urdf_file URDF filename to construct the chain
             */

#if !ARDL_EXTERNAL_DATA
            Chain(std::string urdf_file)
#else
            Chain(std::string urdf_file, ARDL::Data<T> &data)
                : m_links_vsp(data.links), m_movable_links_vsp(data.movable_links), m_joints_all_vsp(data.joints_all),
                  m_joints_vsp(data.joints), m_jointNum(data.jointNum), m_rootName(data.rootName),
                  m_tipName(data.tipName), q(data.q), qd(data.qd), m_data(data)
#endif
            {
                urdf::ModelInterfaceSharedPtr t_model= urdf::parseURDFFile(urdf_file);
                urdf::LinkConstSharedPtr t_rootLink= t_model->getRoot();
                std::string t_rootName= t_rootLink->name;
                urdf::LinkConstSharedPtr t_link;

                if(t_rootName == "world") { t_rootName= t_rootLink->child_links[0]->name; }
                t_link= t_rootLink;
                while(t_link->child_links.size() != 0) { t_link= t_link->child_links[0]; }
                m_rootName= t_rootName;

                std::string t_tipName= t_link->name;
                m_tipName= t_tipName;
                init(t_rootName, t_tipName, t_model);
            }

            /**
             * @brief Construct a new Chain object
             *
             * @param root_name Root link name
             * @param tip_name Tip link name
             * @param urdf_file Filename containing URDF
             */
#if !ARDL_EXTERNAL_DATA
            Chain(std::string root_name, std::string tip_name, std::string urdf_file)
                : Chain(root_name, tip_name, urdf::parseURDFFile(urdf_file))
#else
            Chain(std::string root_name, std::string tip_name, std::string urdf_file, ARDL::Data<T> &data)
                : Chain(root_name, tip_name, urdf::parseURDFFile(urdf_file), data)
#endif
            {
            }

#if !ARDL_EXTERNAL_DATA
            Chain(std::string root_name, std::string tip_name, urdf::ModelInterfaceSharedPtr model)
#else
            Chain(std::string root_name, std::string tip_name, urdf::ModelInterfaceSharedPtr model, ARDL::Data<T> &data)
                : m_links_vsp(data.links), m_movable_links_vsp(data.movable_links), m_joints_all_vsp(data.joints_all),
                  m_joints_vsp(data.joints), m_jointNum(data.jointNum), m_rootName(data.rootName),
                  m_tipName(data.tipName), q(data.q), qd(data.qd), m_data(data)
#endif
            {
                init(root_name, tip_name, model);
            }

            virtual ~Chain() {}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            // Generate random position for each joint
            void randomPos() {
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
                    ARDL_visit(*(m_joints_vsp[t_i]), setRandomQState());
                    this->q.segment(t_i, ARDL_visit(*(m_joints_vsp[t_i]), getDof()))=
                        ARDL_visit(*(m_joints_vsp[t_i]), getQ());
                }
            }

            // Generate random velocity for each joint
            void randomVel() {
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
                    ARDL_visit(*(m_joints_vsp[t_i]), setRandomQdState());
                    this->qd.segment(t_i, ARDL_visit(*(m_joints_vsp[t_i]), getDof()))=
                        ARDL_visit(*(m_joints_vsp[t_i]), getQd());
                }
            }

            void random() {
                randomPos();
                randomVel();
            }

            template<typename qDerived, typename qdDerived>
            void updateChain(const Eigen::MatrixBase<qDerived> &q, const Eigen::MatrixBase<qdDerived> &qd) {
                updateChainPos(q);
                updateChainVel(qd);
            }

            template<typename Derived>
            void updateChainPos(const Eigen::MatrixBase<Derived> &q) {
                // TODO fix for multidof not on t_i
                this->q= q;
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
#if ARDL_VARIANT == ON
    #if VISITORDER
                    ARDL_VISIT(
                        *(m_joints_vsp[t_i]),
                        [&t_i, &q](RevoluteJoint<T> &joint) {
                            joint.setQ(q.template segment<RevoluteJoint<T>::getDof()>(t_i));
                        },
                        [&t_i, &q](FixedJoint<T> &joint) {
                            joint.setQ(q.template segment<FixedJoint<T>::getDof()>(t_i));
                        });
    #else
                    ARDL_VISIT(overloaded{[&t_i, &q](RevoluteJoint<T> &joint) {
                                              joint.setQ(q.template segment<RevoluteJoint<T>::getDof()>(t_i));
                                          },
                                          [&t_i, &q](FixedJoint<T> &joint) {
                                              joint.setQ(q.template segment<FixedJoint<T>::getDof()>(t_i));
                                          }},
                               *(m_joints_vsp[t_i]));
    #endif
#else
                    m_joints_vsp[t_i]->setQ(q.segment(t_i, m_joints_vsp[t_i]->getDof()));
#endif
                }
            }

            template<typename Derived>
            void updateChainVel(const Eigen::MatrixBase<Derived> &qd) {
                this->qd= qd;
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
#if ARDL_VARIANT == ON

    #if VISITORDER
                    ARDL_VISIT(
                        *(m_joints_vsp[t_i]),
                        [&t_i, &qd](RevoluteJoint<T> &joint) {
                            joint.setQd(qd.template segment<RevoluteJoint<T>::getDof()>(t_i));
                        },
                        [&t_i, &qd](FixedJoint<T> &joint) {
                            joint.setQd(qd.template segment<FixedJoint<T>::getDof()>(t_i));
                        });
    #else
                    ARDL_VISIT(overloaded{[&t_i, &qd](RevoluteJoint<T> &joint) {
                                              joint.setQd(qd.template segment<RevoluteJoint<T>::getDof()>(t_i));
                                          },
                                          [&t_i, &qd](FixedJoint<T> &joint) {
                                              joint.setQd(qd.template segment<FixedJoint<T>::getDof()>(t_i));
                                          }},
                               *(m_joints_vsp[t_i]));
    #endif
#else
                    m_joints_vsp[t_i]->setQd(qd.segment(t_i, m_joints_vsp[t_i]->getDof()));
#endif
                }
            }
            void updateMatrices() {
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) { ARDL_visit(*(m_joints_vsp[t_i]), update()); }
            }
            void updateMatricesOptim() {
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) { ARDL_visit(*(m_joints_vsp[t_i]), updateOptim()); }
                ARDL_visit(m_ee_link->getParentJoint(), updateOptim());
            }

            template<typename Derived>
            void updateParams(const Eigen::MatrixBase<Derived> &paramDot) {
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
                    m_movable_links_vsp[t_i + 1]->updateParams(paramDot.template segment(
                        t_i * (paramDot.size() / m_jointNum), (paramDot.size() / m_jointNum)));
                }
            }

            template<typename D>
            void getParams(Eigen::MatrixBase<D> const &params) {
                for(size_t i= 0; i < m_jointNum; i++) {
                    m_movable_links_vsp[i + 1]->getSI().toVector(
                        const_cast<Eigen::MatrixBase<D> &>(params).template block<10, 1>(i * 10, 0));
                }
            }

            unsigned int getNumOfJoints() { return m_jointNum; }

            unsigned int getNumOfLinks() { return m_links_vsp.size(); }

            unsigned int getNumOfMovableLinks() { return m_movable_links_vsp.size(); }

            const aligned_vector<Link<T>> &getLinks() const { return m_links_vsp; }
            const aligned_vector<Link<T> *> &getMoveableLinks() const { return m_movable_links_vsp; }
            aligned_vector<Link<T> *> &getMoveableLinksRef() { return m_movable_links_vsp; }
            const Link<T> &getLink(size_t link_id) { return m_links_vsp[link_id]; }
            JointVariant<T> &getJoint(size_t joint_id) { return *(m_joints_vsp[joint_id]); }
            Link<T>& getEELinkRef() {return *m_ee_link;}

            void getJointLimits(std::vector<std::pair<T, T>> &limits) {
                // TODO Add any length
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
                    double tmp1= ARDL_visit(*(m_joints_vsp[t_i]), getMinLimit())[0],
                           tmp2= ARDL_visit(*(m_joints_vsp[t_i]), getMaxLimit())[0];
                    limits[t_i]= std::make_pair(tmp1, tmp2);
                }
            }
            void getJointLimits(std::pair<VectorX<T>, VectorX<T>> &limits) {
                // TODO Add any length
                limits.first.resize(m_jointNum);
                limits.second.resize(m_jointNum);
                for(size_t t_i= 0; t_i < m_jointNum; t_i++) {
                    double tmp1= ARDL_visit(*(m_joints_vsp[t_i]), getMinLimit())[0],
                           tmp2= ARDL_visit(*(m_joints_vsp[t_i]), getMaxLimit())[0];
                    limits.first(t_i)= tmp1;
                    limits.second(t_i)= tmp2;
                }
            }
            template<typename Derived>
            void getJointVelocityLimits(const Eigen::MatrixBase<Derived> &limits) {
                for(size_t i= 0; i < m_jointNum; i++) {
                    const_cast<Eigen::MatrixBase<Derived> &>(limits)[i]=
                        ARDL_visit(*(m_joints_vsp[i]), getVelLimit())[0];
                }
            }

            aligned_vector<Link<T>> &getLinksRef() { return m_links_vsp; }

            std::string getCollisionFile(size_t link_id) { return m_links_vsp[link_id]->getCollisionMeshFile(); }

            const std::string &getRootName() const { return m_rootName; }
            const std::string &getTipName() const { return m_tipName; }

            const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQ() const { return q; }
            const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQd() const { return qd; }
        };
    } // namespace Model
} // namespace ARDL

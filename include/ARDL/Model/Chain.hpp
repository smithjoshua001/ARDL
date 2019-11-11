#pragma once

#include <string>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "ARDL/Model/Joint.hpp"
#include "ARDL/Model/Link.hpp"
#include "ARDL/Util/Search.hpp"

namespace ARDL {
    namespace Model {
        /**
         * @brief Model of a single chain
         *
         * @tparam T
         */
        template <typename T> class Chain {
        private:
            //List of links (all/movable)
            aligned_vector< std::shared_ptr < Link<T> > > m_links_vsp, m_movable_links_vsp;
            //List of movable joints
            aligned_vector <std::shared_ptr<Joint<T> > > m_joints_all_vsp, m_joints_vsp;
            //Number of movable joints
            size_t m_jointNum;
            //Root name of chain and tip name of chain
            std::string m_rootName, m_tipName;

            //current position and velocity states of the chain
            Eigen::Matrix<T, Eigen::Dynamic, 1> q, qd;

            /**
             * @brief Initialize the chain
             *
             * @param root_name Root link name
             * @param tip_name Tip link name
             * @param model The urdf model of the robot in question
             */
            void init(std::string root_name, std::string tip_name, urdf::ModelInterfaceSharedPtr model) {
                urdf::LinkConstSharedPtr m_rootLink = model->getLink(root_name);

                //Search to build chain between root to tip (designed for multibranching also for failure checking)
                std::stack<urdf::LinkConstSharedPtr> t_urdfChain = Util::Search::searchBreadth(m_rootLink, [] (urdf::LinkConstSharedPtr l, std::queue<urdf::LinkConstSharedPtr> &q, std::map<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr> &m) {
                                                                                                   for (urdf::LinkConstSharedPtr childL: l->child_links) {
                                                                                                       if (m.find(childL) != m.end()) {
                                                                                                           continue;
                                                                                                       }
                                                                                                       q.push(childL);
                                                                                                       m.insert(std::pair<urdf::LinkConstSharedPtr, urdf::LinkConstSharedPtr>(childL, l));
                                                                                                   }

                                                                                                   return;
                                                                                               }, tip_name);

                m_rootName = root_name;
                m_tipName = tip_name;

                if (t_urdfChain.size() == 0) {
                    throw std::runtime_error("URDF is of size 0");
                } else {
                    m_jointNum = 0;
                    this->m_links_vsp.push_back(std::shared_ptr<Link<T> >(new Link<T>(t_urdfChain.top(), true, nullptr)));
                    this->m_links_vsp.back()->loadChildJoints(t_urdfChain.top());
                    t_urdfChain.pop();
                    while (!t_urdfChain.empty()) {
                        this->m_links_vsp.push_back(std::shared_ptr<Link<T> >(new Link<T>(t_urdfChain.top(), false, this->m_links_vsp.back())));
                        this->m_links_vsp.back()->loadChildJoints(t_urdfChain.top());
                        if (!this->m_links_vsp.back()->getParentJoint()->isFixed()) {
                            m_jointNum++;
                            this->m_movable_links_vsp.push_back(this->m_links_vsp.back());
                            this->m_joints_vsp.push_back(this->m_links_vsp.back()->getParentJoint());
                        }
                        this->m_joints_all_vsp.push_back(this->m_links_vsp.back()->getParentJoint());
                        t_urdfChain.pop();
                    }
                    q.resize(m_jointNum);
                    qd.resize(m_jointNum);
                }
            }

        private:
            size_t t_i;

        public:

            Chain(const Chain<T> &copy) {
                m_jointNum = copy.m_jointNum;
                m_rootName = copy.m_rootName;
                m_tipName = copy.m_tipName;
                q = copy.q;
                qd = copy.qd;

                m_links_vsp.push_back(std::move(std::shared_ptr<Link<T> >(new Link<T>(*(copy.m_links_vsp[0]), nullptr, nullptr))));
                for (size_t i = 1; i < copy.m_links_vsp.size(); i++) {
                    std::shared_ptr<RevoluteJoint<T> > joint_ptr = std::dynamic_pointer_cast<RevoluteJoint<T> >(copy.m_joints_all_vsp[i - 1]);
                    if (joint_ptr) {
                        m_joints_all_vsp.push_back(std::move(std::shared_ptr<Joint<T> >(new RevoluteJoint<T>(*(joint_ptr), m_links_vsp[i - 1]))));
                        m_joints_vsp.push_back(m_joints_all_vsp.back());
                    } else {
                        std::shared_ptr<FixedJoint<T> > fixed_ptr = std::dynamic_pointer_cast<FixedJoint<T> >(copy.m_joints_all_vsp[i - 1]);
                        if (fixed_ptr) {
                            m_joints_all_vsp.push_back(std::move(std::shared_ptr<Joint<T> >(new FixedJoint<T>(*(fixed_ptr), m_links_vsp[i - 1]))));
                        }
                    }

                    m_links_vsp.push_back(std::move(std::shared_ptr<Link<T> >(new Link<T>(*(copy.m_links_vsp[i]), m_joints_all_vsp[i - 1], m_links_vsp[i - 1]))));
                    if (joint_ptr) {
                        m_movable_links_vsp.push_back(m_links_vsp.back());
                    }
                }
            }

            /**
             * @brief Construct a new Chain object
             * Only works with single chain robots.
             *
             * @param urdf_file URDF filename to construct the chain
             */
            Chain(std::string urdf_file) {
                urdf::ModelInterfaceSharedPtr t_model = urdf::parseURDFFile(urdf_file);
                urdf::LinkConstSharedPtr t_rootLink = t_model->getRoot();
                std::string t_rootName = t_rootLink->name;
                urdf::LinkConstSharedPtr t_link;

                if (t_rootName == "world") {
                    t_rootName = t_rootLink->child_links[0]->name;
                }
                t_link = t_rootLink;
                while (t_link->child_links.size() != 0) {
                    t_link = t_link->child_links[0];
                }
                m_rootName = t_rootName;

                std::string t_tipName = t_link->name;
                m_tipName = t_tipName;
                init(t_rootName, t_tipName, t_model);
            }
            /**
             * @brief Construct a new Chain object
             *
             * @param root_name Root link name
             * @param tip_name Tip link name
             * @param urdf_file Filename containing URDF
             */
            Chain(std::string root_name, std::string tip_name, std::string urdf_file) : Chain(root_name, tip_name, urdf::parseURDFFile(urdf_file)) {}
            Chain(std::string root_name, std::string tip_name, urdf::ModelInterfaceSharedPtr model) {
                init(root_name, tip_name, model);
            }

            virtual ~Chain() {}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            aligned_vector < std::shared_ptr < Link<T> > > &getLinksRef() {
                return m_links_vsp;
            }

            //Generate random position for each joint
            void randomPos() {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->setRandomQState();
                    this->q(t_i) = m_joints_vsp[t_i]->getQ();
                }
            }

            //Generate random velocity for each joint
            void randomVel() {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->setRandomQdState();
                    this->qd(t_i) = m_joints_vsp[t_i]->getQd();
                }
            }

            void random() {
                randomPos();
                randomVel();
            }

            template <typename qDerived, typename qdDerived> void updateChain(const Eigen::MatrixBase<qDerived> &q, const Eigen::MatrixBase<qdDerived> &qd) {
                updateChainPos(q);
                updateChainVel(qd);
            }

            template <typename Derived> void updateChainPos(const Eigen::MatrixBase<Derived> &q) {
                this->q = q;
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->setQ(q(t_i));
                }
            }

            template <typename Derived> void updateChainVel(const Eigen::MatrixBase<Derived> &qd) {
                this->qd = qd;
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->setQd(qd(t_i));
                }
            }

            void updateMatrices() {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->update();
                }
            }
            void updateMatricesOptim() {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_joints_vsp[t_i]->updateOptim();
                }
            }

            template <typename Derived> void updateParams(const Eigen::MatrixBase<Derived> &paramDot) {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    m_movable_links_vsp[t_i]->updateParams(paramDot.template segment(t_i * (paramDot.size() / m_jointNum), (paramDot.size() / m_jointNum)));
                }
                // size_t j = 0;
                // for (std::shared_ptr<Link<T> > link: m_links_vsp) {
                //     if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                //         link->updateParams(paramDot.template segment(j * (paramDot.size() / getNumOfJoints()), (paramDot.size() / getNumOfJoints())));
                //         j++;
                //     }
                // }
            }

            // template <typename Derived> void getFrictionParams(const Eigen::MatrixBase<Derived> &params) {
            //     size_t j = 0;
            //     for (std::shared_ptr<Link<T> > link: m_links_vsp) {
            //         if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
            //             const_cast < Eigen::MatrixBase < Derived > & > (params).template block < 2, 1 > (j * 2, 0) = link->getInertialParams().block < 2, 1 > (10, 0);
            //             j++;
            //         }
            //     }

            //     assert(j == getNumOfJoints());
            // }

            // template < typename Derived > void getParams(Eigen::MatrixBase < Derived > const &params) {
            //     size_t j = 0;
            //     for (std::shared_ptr<Link<T> > link: m_links_vsp) {
            //         if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
            //             const_cast < Eigen::MatrixBase < Derived > & > (params).template block < 10, 1 > (j * 10, 0) = link->getInertialParams().template block < 10, 1 > (0, 0);
            //             j++;
            //         }
            //     }

            //     assert(j == getNumOfJoints());
            // }

            template < typename Derived > void getParams(Eigen::MatrixBase < Derived > const &params) {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    const_cast < Eigen::MatrixBase < Derived > & > (params).template block < 12, 1 > (t_i * 12, 0) = m_movable_links_vsp[t_i]->getInertialParams();
                }
                // size_t j = 0;
                // for (std::shared_ptr<Link<T> > link: m_links_vsp) {
                //     if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                //         const_cast < Eigen::MatrixBase < Derived > & > (params).template block < 12, 1 > (j * 12, 0) = link->getInertialParams();
                //         j++;
                //     }
                // }

                // assert(j == getNumOfJoints());
            }

            unsigned int getNumOfJoints() {
                return m_jointNum;
            }

            unsigned int getNumOfLinks() {
                return m_links_vsp.size();
            }

            unsigned int getNumOfMovableLinks() {
                return m_movable_links_vsp.size();
            }

            const aligned_vector< std::shared_ptr < Link<T> > > &getLinks() const {
                return m_links_vsp;
            }
            const aligned_vector < std::shared_ptr < Link<T> > > &getMovableLinks() const {
                return m_movable_links_vsp;
            }

            std::string getCollisionFile(size_t link_id) {
                return m_links_vsp[link_id]->getCollisionMeshFile();
            }

            const std::shared_ptr < Link<T> > getLink(size_t link_id) {
                return m_links_vsp[link_id];
            }

            void getJointLimits(std::vector<std::pair<T, T> > &limits) {
                for (t_i = 0; t_i < m_jointNum; t_i++) {
                    limits[t_i] = std::make_pair<T, T>(m_joints_vsp[t_i]->getMinLimit(), m_joints_vsp[t_i]->getMaxLimit());
                }
                // size_t j = 0;
                // for (std::shared_ptr<Link<T> > link: m_links_vsp) {
                //     if (!link->isRoot() && !link->getParentJoint()->isFixed()) {
                //         limits.at(j) = std::make_pair<T, T>(link->getParentJoint()->getMinLimit(), link->getParentJoint()->getMaxLimit());
                //         j++;
                //     }
                // }
            }

            std::shared_ptr<Joint<T> > getJoint(size_t joint_id) {
                return m_joints_vsp[joint_id];
            }

            const std::string &getRootName() const {
                return m_rootName;
            }
            const std::string &getTipName() const {
                return m_tipName;
            }

            const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQ() const {
                return q;
            }
            const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQd() const {
                return qd;
            }
        };
    }
}

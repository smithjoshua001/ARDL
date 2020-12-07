#pragma once

#include "ARDL/Model/Joint.hpp"
#include "ARDL/Model/Joints/Joints.hpp"
#include "ARDL/Model/Link.hpp"
#include "ARDL/Util/Overloaded.hpp"
#include <stack>
#include <string>
#include <unordered_map>
#include <urdf_model/model.h>
#include <urdf_model/types.h>
#include <urdf_parser/urdf_parser.h>

namespace ARDL::Model {
/**
 * @brief Model of a tree platform
 *
 * @tparam T Precision type (float/double)
 */
template <typename T> class Tree {
  private:
    // List of links (all/movable)
    aligned_vector<Link<T>> m_links;
    aligned_vector<Link<T> *> m_eeLinks;
    aligned_vector<Link<T> *> m_moveableLinks;
    std::unordered_map<Link<T> *, size_t> m_parentMap, m_moveableParentMap;
    // List of movable joints
    aligned_vector<JointVariant<T>> m_joints;
    aligned_vector<JointVariant<T> *> m_moveableJoints;
    // Number of movable joints
    size_t m_jointNum;
    // Root name
    std::string m_rootName;

    std::vector<std::string> m_eeNames;

    // current position and velocity states of the chain
    Eigen::Matrix<T, Eigen::Dynamic, 1> m_q, m_qd;

    /**
   * @brief Initialize the chain
   *
   * @param root_name Root link name
   * @param tip_name Tip link name
   * @param model The urdf model of the robot in question
   */
    void init(std::string root_name, urdf::ModelInterfaceSharedPtr model) {
        m_links.reserve(model->links_.size());
        m_joints.reserve(model->joints_.size());
        urdf::LinkConstSharedPtr m_rootLink = model->getLink(root_name);

        // set current joint numbers to 0
        m_jointNum = 0;

        // Place root link into link vector (no parent link or joint)
        this->m_links.emplace_back(m_rootLink, nullptr, nullptr);
        // Depth first expansion
        std::stack<std::pair<urdf::LinkConstSharedPtr, Link<T> &>> dfs;
        // Load child joints of root into expansion (store parent link due to tree
        // structrue)
        // for (const auto &c : m_rootLink->child_links) {

        for (auto it = m_rootLink->child_links.rbegin();
             it != m_rootLink->child_links.rend(); it++) {
            dfs.push({*it, this->m_links.back()});
        }
        // push root into moveable links to allow static transform to be added
        this->m_moveableLinks.push_back(&(this->m_links.back()));
        // Expand until all links have been expanded
        while (!dfs.empty()) {
            std::pair<urdf::LinkConstSharedPtr, Link<T> &> top = dfs.top();

            dfs.pop();
            // Load child joint from parent link to current link
            top.second.loadChildJoint(top.first->name, m_joints);
            // create ARDL link from current link, parent link and the loaded joint
            this->m_links.emplace_back(top.first, &(top.second),
                                       &(this->m_joints.back()));
            // create parent index map
            auto it = std::find(this->m_links.begin(), this->m_links.end(),
                                top.second);
            m_parentMap[&(this->m_links.back())] = it - this->m_links.begin();

            //find movable parent
            Link<T> *parentMoveable = &(top.second);
            while (!parentMoveable->isRoot() &&
                   ARDL_visit(parentMoveable->getParentJoint(), isFixed())) {
                parentMoveable = &(this->m_links[m_parentMap[parentMoveable]]);
            }

            // Push all the child joint from current link into expansion
            // for (urdf::LinkConstSharedPtr c : top.first->child_links) {
            for (auto it = top.first->child_links.rbegin();
                 it != top.first->child_links.rend(); it++) {
                dfs.push({*it, this->m_links.back()});
            }
            // If there is no more children then this is one of the end effector links
            // (possibly need a map)
            if (top.first->child_links.size() == 0) {
                m_eeLinks.push_back(&(this->m_links.back()));
                m_eeNames.push_back(this->m_eeLinks.back()->getName());
            }
            // if joint not fixed then put into moveable joints/links/add one to the
            // joint number
            if (!ARDL_visit(this->m_links.back().getParentJoint(), isFixed())) {
                m_jointNum++;
                // this->m_moveableParentMap[&(this->m_links.back())] =
                //     this->m_moveableLinks.size() - 1;
                this->m_moveableLinks.push_back(&(this->m_links.back()));
                this->m_moveableJoints.push_back(
                    &(this->m_links.back().getParentJoint()));

                auto it2 =
                    std::find(this->m_moveableLinks.begin(),
                              this->m_moveableLinks.end(), parentMoveable);
                m_moveableParentMap[this->m_moveableLinks.back()] =
                    it2 - this->m_moveableLinks.begin();
            } else if (top.first->child_links.size() == 0 &&
                       ARDL_visit(this->m_links.back().getParentJoint(),
                                  isFixed())) {

                auto it2 =
                    std::find(this->m_moveableLinks.begin(),
                              this->m_moveableLinks.end(), parentMoveable);
                m_moveableParentMap[&(this->m_links.back())] =
                    it2 - this->m_moveableLinks.begin();
            }

            // create moveable parent index map
            // if (!parentMoveable->isRoot()) {
            // }
        }
        // set size of q and qd for storage
        m_q.resize(m_jointNum);
        m_qd.resize(m_jointNum);
    }

  public:
    /**
   * @brief Copy constructor for chain
   *
   * @param copy The chain to copy
   */
    Tree(const Tree<T> &copy) {
        m_jointNum = copy.m_jointNum;
        m_rootName = copy.m_rootName;
        m_eeNames = copy.m_eeNames;
        m_q = copy.m_q;
        m_qd = copy.m_qd;

        m_links.emplace_back(copy.m_links[0], nullptr, nullptr);
        for (size_t i = 1; i < copy.m_links.size(); i++) {
            std::ptrdiff_t parentLIndex =
                &(copy.m_links[i].getParentLink()) - copy.m_links.data();
            std::ptrdiff_t parentJIndex =
                &(copy.m_links[i].getParentJoint()) - copy.m_joints.data();
            // Copy joints, and assigns parent joints and links using offset from old
            // vector to new vector
#if ARDL_VARIANT
#if VISITORDER
            ARDL_VISIT(
                copy.m_joints[i - 1],
                [&](RevoluteJoint<T> &joint) {
                    m_joints.emplace_back(
                        RevoluteJoint<T>(joint, m_links[parentLIndex]));
                },
                [&](PrismaticJoint<T> &joint) {
                    m_joints.emplace_back(
                        PrismaticJoint<T>(joint, m_links[parentLIndex]));
                },
                [&](FixedJoint<T> &joint) {
                    m_joints.emplace_back(
                        FixedJoint<T>(joint, m_links[parentLIndex]));
                });
#else
            ARDL_VISIT(overloaded{[&](const RevoluteJoint<T> &joint) {
                                      m_joints.emplace_back(RevoluteJoint<T>(
                                          joint, m_links[parentLIndex]));
                                  },
                                  [&](const PrismaticJoint<T> &joint) {
                                      m_joints.emplace_back(PrismaticJoint<T>(
                                          joint, m_links[parentLIndex]));
                                  },
                                  [&](const FixedJoint<T> &joint) {
                                      m_joints.emplace_back(FixedJoint<T>(
                                          joint, m_links[parentLIndex]));
                                  }},
                       copy.m_joints[i - 1]);
#endif
            m_links.emplace_back(copy.m_links[i], &(m_joints[parentJIndex]),
                                 &(m_links[parentLIndex]));

#else
            std::shared_ptr<RevoluteJoint<T>> joint_ptr =
                std::dynamic_pointer_cast<RevoluteJoint<T>>(
                    &(copy.m_joints[i - 1]));
            if (joint_ptr) {
                m_joints.push_back(
                    RevoluteJoint<T>(*(joint_ptr), &(m_links[parentLIndex])));
                m_moveableJoints.push_back(&(m_joints.back()));
            } else {
                std::shared_ptr<FixedJoint<T>> fixed_ptr =
                    std::dynamic_pointer_cast<FixedJoint<T>>(
                        &(copy.m_joints[i - 1]));
                if (fixed_ptr) {
                    m_joints.push_back(
                        FixedJoint<T>(*(joint_ptr), m_links[parentLIndex]));
                }
            }
            m_links.emplace_back(copy.m_links[0], &(m_joints.back()),
                                 &(m_links[parentLIndex]));
            if (joint_ptr) {
                m_moveableLinks.push_back(&(m_links.back()));
            }
#endif
        }
        for (size_t i = 0; i < copy.m_moveableLinks.size(); i++) {
            std::ptrdiff_t parentLIndex =
                copy.m_moveableLinks[i] - copy.m_links.data();
            m_moveableLinks.push_back(m_links.data() + parentLIndex);
        }
        for (size_t i = 0; i < copy.m_moveableJoints.size(); i++) {
            std::ptrdiff_t parentJIndex =
                copy.m_moveableJoints[i] - copy.m_joints.data();
            m_moveableJoints.push_back(m_joints.data() + parentJIndex);
        }
    }

    /**
   * @brief Construct a new Chain object
   * Only works with single chain robots.
   *
   * @param urdf_file URDF filename to construct the chain
   */
    Tree(std::string urdf_file) {
        // Parse the URDF
        urdf::ModelInterfaceSharedPtr t_model = urdf::parseURDFFile(urdf_file);
        // Get the root
        urdf::LinkConstSharedPtr t_rootLink = t_model->getRoot();
        std::string t_rootName = t_rootLink->name;
        // Temporary link for finding the tip
        urdf::LinkConstSharedPtr t_link;
        // If the root is world (commonly found in gazebo I think) move to next
        // joint
        if (t_rootName == "world") {
            t_rootName = t_rootLink->child_links[0]->name;
        }
        t_link = t_rootLink;
        while (t_link->child_links.size() != 0) {
            t_link = t_link->child_links[0];
        }
        m_rootName = t_rootName;

        // init chain from root and found tip
        init(t_rootName, t_model);
    }

    Tree(std::string root_name, std::string urdf_file)
        : Tree(root_name, urdf::parseURDFFile(urdf_file)) {}
    Tree(std::string root_name, std::string tip_name,
         urdf::ModelInterfaceSharedPtr model) {
        init(root_name, tip_name, model);
    }

    virtual ~Tree() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
   * @brief Generate a random position of the robot.
   *
   */
    void randomPos() {
        size_t currentIndex = 0;
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            // for all movable joints set a random q state according to the joint type
            ARDL_visit(*(m_moveableJoints[t_i]), setRandomQState());
            // get the random q from the joint and put it in the state vector for the
            // chain
            this->m_q.segment(currentIndex,
                              ARDL_visit(*(m_moveableJoints[t_i]), getDof())) =
                ARDL_visit(*(m_moveableJoints[t_i]), getQ());
            // move current index by the dof of the joint (for future use when joints
            // can be multi-dof)
            currentIndex += ARDL_visit(*(m_moveableJoints[t_i]), getDof());
        }
    }

    /**
   * @brief Generate a random velocity of the robot.
   *
   */
    void randomVel() {
        size_t currentIndex = 0;
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            // for all movable joints set a random qd state according to the joint
            // type
            ARDL_visit(*(m_moveableJoints[t_i]), setRandomQdState());
            // get the random qd from the joint and put it in the state vector for the
            // chain
            this->m_qd.segment(t_i,
                               ARDL_visit(*(m_moveableJoints[t_i]), getDof())) =
                ARDL_visit(*(m_moveableJoints[t_i]), getQd());
            // move current index by the dof of the joint (for future use when joints
            // can be multi-dof)
            currentIndex += ARDL_visit(*(m_moveableJoints[t_i]), getDof());
        }
    }

    /**
   * @brief Generate random configuration of the chain (position and velocity)
   *
   */
    void random() {
        randomPos();
        randomVel();
    }

    /**
   * @brief Update the chain with a new position and velocity
   *
   * @tparam qDerived Derived type of q so that we can pass full vectors, or
   * segments
   * @tparam qdDerived Derived type of qd so that we can pass full vectors or
   * segments
   * @param q Position of each joint
   * @param qd Velocity of each joint
   */
    template <typename qDerived, typename qdDerived>
    void update(const Eigen::MatrixBase<qDerived> &q,
                const Eigen::MatrixBase<qdDerived> &qd) {
        updatePos(q);
        updateVel(qd);
    }
    /**
   * @brief Update the chain with a new position
   *
   * @tparam Derived Derived type of q so that we can pass full vectors, or
   * segments
   * @param q Position of each joint
   */
    template <typename Derived>
    void updatePos(const Eigen::MatrixBase<Derived> &q) {
        size_t q_i = 0;
        // TODO fix for multidof not on t_i
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
#if ARDL_VARIANT
#if VISITORDER
            ARDL_VISIT(
                *(m_moveableJoints[t_i]),
                [&t_i, &q, &q_i](RevoluteJoint<T> &joint) {
                    joint.setQ(
                        q.template segment<RevoluteJoint<T>::getDof()>(q_i));
                    q_i += RevoluteJoint<T>::getDof();
                },
                [&t_i, &q, &q_i](PrismaticJoint<T> &joint) {
                    joint.setQ(
                        q.template segment<PrismaticJoint<T>::getDof()>(q_i));
                    q_i += PrismaticJoint<T>::getDof();
                },
                [&t_i, &q](FixedJoint<T> &joint) {
                    //     joint.setQ(q.template segment<FixedJoint<T>::getDof()>(t_i));
                    //   t_i+=FixedJoint<T>::getDof();
                });
#else
            ARDL_VISIT(
                overloaded{
                    [&t_i, &q, &q_i](RevoluteJoint<T> &joint) {
                        joint.setQ(
                            q.template segment<RevoluteJoint<T>::getDof()>(
                                q_i));
                        q_i += RevoluteJoint<T>::getDof();
                    },
                    [&t_i, &q, &q_i](PrismaticJoint<T> &joint) {
                        joint.setQ(
                            q.template segment<PrismaticJoint<T>::getDof()>(
                                q_i));
                        q_i += PrismaticJoint<T>::getDof();
                    },
                    [&t_i, &q](FixedJoint<T> &joint) {
                        joint.setQ(
                            q.template segment<FixedJoint<T>::getDof()>(t_i));
                    }},
                *(m_moveableJoints[t_i]));
#endif
#else
            std::cout << "Type3" << std::endl;
            m_moveableJoints[t_i]->setQ(
                q.segment(t_i, m_moveableJoints[t_i]->getDof()));
#endif
        }
        this->m_q = q;
    }
    /**
   * @brief Update the chain with a new velocity
   *
   * @tparam Derived Derived type of qd so that we can pass full vectors, or
   * segments
   * @param qd Velocity of each joint
   */
    template <typename Derived>
    void updateVel(const Eigen::MatrixBase<Derived> &qd) {
        size_t q_i = 0;
        this->m_qd = qd;
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
#if ARDL_VARIANT

#if VISITORDER
            ARDL_VISIT(
                *(m_moveableJoints[t_i]),
                [&t_i, &qd, &q_i](RevoluteJoint<T> &joint) {
                    joint.setQd(
                        qd.template segment<RevoluteJoint<T>::getDof()>(q_i));
                    q_i += RevoluteJoint<T>::getDof();
                },
                [&t_i, &qd, &q_i](PrismaticJoint<T> &joint) {
                    joint.setQd(
                        qd.template segment<PrismaticJoint<T>::getDof()>(q_i));
                    q_i += PrismaticJoint<T>::getDof();
                },
                [&t_i, &qd](FixedJoint<T> &joint) {
                    joint.setQd(
                        qd.template segment<FixedJoint<T>::getDof()>(t_i));
                });
#else
            ARDL_VISIT(
                overloaded{
                    [&t_i, &qd, &q_i](RevoluteJoint<T> &joint) {
                        joint.setQd(
                            qd.template segment<RevoluteJoint<T>::getDof()>(
                                q_i));
                        q_i += RevoluteJoint<T>::getDof();
                    },
                    [&t_i, &qd, &q_i](PrismaticJoint<T> &joint) {
                        joint.setQd(
                            qd.template segment<PrismaticJoint<T>::getDof()>(
                                q_i));
                        q_i += PrismaticJoint<T>::getDof();
                    },
                    [&t_i, &qd](FixedJoint<T> &joint) {
                        joint.setQd(
                            qd.template segment<FixedJoint<T>::getDof()>(t_i));
                    }},
                *(m_moveableJoints[t_i]));
#endif
#else
            m_moveableJoints[t_i]->setQd(
                qd.segment(t_i, m_moveableJoints[t_i]->getDof()));
#endif
        }
    }
    /**
   * @brief Update the internal matrices needed after any updates
   *
   */
    void updateMatrices() {
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            ARDL_visit(*(m_moveableJoints[t_i]), update());
        }
    }
    /**
   * @brief Update the optimal internal matrices needed after any updates
   * (Optimal such that only movable links and joints are updated)
   *
   */
    void updateMatricesOptim() {
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            ARDL_visit(*(m_moveableJoints[t_i]), updateOptim());
        }
        for (size_t t_i = 0; t_i < m_eeLinks.size(); t_i++) {
            ARDL_visit(m_eeLinks[t_i]->getParentJoint(), updateOptim());
        }
    }

    /**
   * @brief Update internal parameters by a parameter change
   *
   * @tparam Derived Derived type of paramDot to allow full vectors or segments
   * @param paramDot Change in parameters to add to existing parameters
   */
    template <typename Derived>
    void updateParams(const Eigen::MatrixBase<Derived> &paramDot) {
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            m_moveableLinks[t_i + 1]->updateParams(
                paramDot.template segment(t_i * (paramDot.size() / m_jointNum),
                                          (paramDot.size() / m_jointNum)));
        }
    }

    /**
   * @brief Get the inertial parameters of the chain
   *
   * @tparam D Derived type of the input/output so that we can pass full vectors
   * or segments
   * @param params The vector to store the inertial parameters in
   */
    template <typename D> void getParams(Eigen::MatrixBase<D> const &params) {
        for (size_t i = 0; i < m_jointNum; i++) {
            m_moveableLinks[i + 1]->getSI().toVector(
                const_cast<Eigen::MatrixBase<D> &>(params)
                    .template block<10, 1>(i * 10, 0));
        }
    }
    /**
   * @brief Get the number of joints (Movable)
   *
   * @return unsigned int The number of joints
   */
    unsigned int getNumOfJoints() { return m_jointNum; }

    /**
   * @brief Get the total number of all links
   *
   * @return unsigned int The number of all links
   */
    unsigned int getNumOfLinks() { return m_links.size(); }

    /**
   * @brief Get the number of links with movable joints
   *
   * @return unsigned int The number of links with moveable joints
   */
    unsigned int getNumOfMovableLinks() { return m_moveableLinks.size(); }

    const aligned_vector<Link<T>> &getLinks() const { return m_links; }
    const aligned_vector<Link<T> *> &getMoveableLinks() const {
        return m_moveableLinks;
    }
    aligned_vector<Link<T> *> &getMoveableLinksRef() { return m_moveableLinks; }
    const Link<T> &getLink(size_t link_id) { return m_links[link_id]; }
    JointVariant<T> &getJoint(size_t joint_id) {
        return *(m_moveableJoints[joint_id]);
    }
    aligned_vector<Link<T> *> &getEELinksRef() { return m_eeLinks; }

    void getJointLimits(std::vector<std::pair<T, T>> &limits) {
        // TODO Add any length
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            double tmp1 =
                       ARDL_visit(*(m_moveableJoints[t_i]), getMinLimit())[0],
                   tmp2 =
                       ARDL_visit(*(m_moveableJoints[t_i]), getMaxLimit())[0];
            limits[t_i] = std::make_pair(tmp1, tmp2);
        }
    }
    void getJointLimits(std::pair<VectorX<T>, VectorX<T>> &limits) {
        // TODO Add any length
        limits.first.resize(m_jointNum);
        limits.second.resize(m_jointNum);
        for (size_t t_i = 0; t_i < m_jointNum; t_i++) {
            double tmp1 =
                       ARDL_visit(*(m_moveableJoints[t_i]), getMinLimit())[0],
                   tmp2 =
                       ARDL_visit(*(m_moveableJoints[t_i]), getMaxLimit())[0];
            limits.first(t_i) = tmp1;
            limits.second(t_i) = tmp2;
        }
    }
    template <typename Derived>
    void getJointVelocityLimits(const Eigen::MatrixBase<Derived> &limits) {
        for (size_t i = 0; i < m_jointNum; i++) {
            const_cast<Eigen::MatrixBase<Derived> &>(limits)[i] =
                ARDL_visit(*(m_moveableJoints[i]), getVelLimit())[0];
        }
    }

    aligned_vector<Link<T>> &getLinksRef() { return m_links; }

    std::string getCollisionFile(size_t link_id) {
        return m_links[link_id]->getCollisionMeshFile();
    }

    const std::string &getRootName() const { return m_rootName; }
    const std::vector<std::string> &getEENames() const { return m_eeNames; }

    const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQ() const { return m_q; }
    const Eigen::Matrix<T, Eigen::Dynamic, 1> &getQd() const { return m_qd; }

    const size_t getMoveableParentId(Link<T> *link) {
        return m_moveableParentMap[link];
    }
};
} // namespace ARDL::Model

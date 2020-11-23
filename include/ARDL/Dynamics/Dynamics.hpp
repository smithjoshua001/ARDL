#pragma once

#include "ARDL/typedefs.hpp"
#include "ARDL/Model/Chain.hpp"
#include "ARDL/Kinematics/ForwardKinematics.hpp"
#include "ARDL/Util/Math.hpp"
#include "ARDL/Util/Filter.hpp"
#include "ARDL/Dynamics/LinkRegressors.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"
#include "ARDL/Util/sobol.hpp"
namespace ARDL {
    using namespace Model;
    using namespace Math;
    using namespace Util::Filters;
    using namespace Util::Math;
    using namespace Regressors;
    template<typename T>
    class Dynamics {
         private:
        std::shared_ptr<Chain<T>> m_chain;

        Gravity<T> m_gravity, mt_gravity;

        Motion<T> mt_bodyVelocity, mt_tempVelocity;

        LinkRegressor<T> mt_momentumRegressor, mt_momentumRegressor2, mt_momentumRegressor3;

        size_t m_baseParamNo;

        Regressor<T> m_projRegressor, m_projParameters;
        MatrixX<T> m_regressorProjected;

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mt_6xJ;

        Eigen::Matrix<T, 6, 6> mt_6x6_1, mt_6x6_2, mt_6x6_3;

        SpatialInertia<T> mt_sI;
        Pose<T> mt_accumulatorGlobal;

         public:
        /**
         * @brief Construct a new Dynamics object by copying
         *
         * @param copy Dynamics to copy
         * @param chain New chain to point to if neccessary
         */
        Dynamics(const Dynamics<T> &copy, std::shared_ptr<Chain<T>> chain) {
            m_chain= chain;
            m_gravity= copy.m_gravity;
            mt_gravity= copy.mt_gravity;
            mt_bodyVelocity= copy.mt_bodyVelocity;
            mt_tempVelocity= copy.mt_tempVelocity;
            mt_momentumRegressor= copy.mt_momentumRegressor;
            mt_momentumRegressor2= copy.mt_momentumRegressor2;
            mt_momentumRegressor3= copy.mt_momentumRegressor3;
            m_baseParamNo= copy.m_baseParamNo;
            m_projRegressor= copy.m_projRegressor;
            m_projParameters= copy.m_projParameters;
            mt_6xJ= copy.mt_6xJ;
            mt_6x6_1= copy.mt_6x6_1;
            mt_6x6_2= copy.mt_6x6_2;
        }

        /**
         * @brief Construct a new Dynamics object
         *
         * @param chain Robot Model (Chain)
         * @param grav Global gravity vector in origin
         */
        Dynamics(std::shared_ptr<Chain<T>> &chain, Gravity<T> &grav) {
            this->m_chain= chain;
            this->m_gravity= grav;
            mt_6xJ.resize(6, this->m_chain->getNumOfJoints());
        }

        /**
         * @brief Construct a new Dynamics object
         *
         * @param chain Robot Model (Chain)
         */
        Dynamics(std::shared_ptr<Chain<T>> &chain) {
            this->m_chain= chain;
            this->m_gravity.setZero();
            this->m_gravity(2)= 9.81;
            mt_6xJ.resize(6, this->m_chain->getNumOfJoints());
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         * @brief Get the Chain object
         *
         * @return std::shared_ptr<Chain<T>>& Pointer to robot model
         */
        std::shared_ptr<Chain<T>> &getChain() { return m_chain; }

        /**
         * @brief Set global acceleration due to gravity
         *
         * @param gravity 6D vector of global gravity
         */
        void setGravity(const Gravity<T> &gravity) { this->m_gravity= gravity; }

        /**
         * @brief Calculate the Joint Inertia matrix
         *
         * @tparam InertiaD Subtype of inertia matrix (allow for block)
         * @param jacobians Aligned vector of jacobians
         * @param M Inertia matrix
         */
        template<Frame OF= Frame::BODY, typename InertiaD>
        void calcJointInertiaMatrix(const aligned_vector<Pose<T>> Ads,
                                    const aligned_vector<Jacobian<T>> &jacobians,
                                    Eigen::MatrixBase<InertiaD> const &M) {
            size_t i= 0;
            const_cast<Eigen::MatrixBase<InertiaD> &>(M).setZero();
            for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                    mt_sI= link->getSI();
                    if constexpr(OF == Frame::SPATIAL) { mt_sI.applyInverseXIX(Ads[i]); }
                    mt_6xJ.noalias()= mt_sI.calculateSpatialInertia() * jacobians[i];
                    const_cast<Eigen::MatrixBase<InertiaD> &>(M).noalias()+= jacobians[i].transpose() * mt_6xJ;
                    i++;
                }
            }
        }

        /**
         * @brief Calculate the Joint Inertia matrix
         *
         * @tparam InertiaD Subtype of inertia matrix (allow for block)
         * @param jacobians Aligned vector of jacobians
         * @param M Inertia matrix
         */
        template<Frame OF= Frame::BODY, typename InertiaD>
        void calcJointInertiaMatrixOptim(const aligned_vector<Pose<T>> Ads,
                                         const aligned_vector<Jacobian<T>> &jacobians,
                                         Eigen::MatrixBase<InertiaD> const &M) {
            const_cast<Eigen::MatrixBase<InertiaD> &>(M).setZero();
            for(size_t i= 0; i < m_chain->getMoveableLinks().size() - 1; i++) {
                mt_sI= m_chain->getMoveableLinks()[i + 1]->getSI();
                if constexpr(OF == Frame::SPATIAL) { mt_sI.applyInverseXIX(Ads[i]); }
                mt_6xJ.noalias()= mt_sI.calculateSpatialInertia() * jacobians[i];
                const_cast<Eigen::MatrixBase<InertiaD> &>(M).noalias()+= jacobians[i].transpose() * mt_6xJ;
            }
        }

        /**
         * @brief Calculate Coriolis and centrifugal matrix
         *
         * @tparam Derived Subtype of output matrix
         * @param jacobians Aligned Vector of jacobians
         * @param jacobianDots Aligned vector of the derivative of the jacobians
         * @param lbSE3 Lie Bracket for each joint
         * @param C Output Coriolis and centrifugal matrix
         */
        template<Frame OF= Frame::BODY, typename Derived>
        void calcCoriolisMatrix(aligned_vector<Jacobian<T>> &jacobians, aligned_vector<Jacobian<T>> &jacobianDots,
                                aligned_vector<Motion<T>> &lbSE3, aligned_vector<Pose<T>> &Ads,
                                Eigen::MatrixBase<Derived> const &C) {
            const_cast<Eigen::MatrixBase<Derived> &>(C).setZero();
            size_t i= 0;
            for(Link<T> &link: m_chain->getLinksRef()) {
                // if constexpr(OF == Frame::SPATIAL) {
                //     if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                //         mt_sI= link->getSI();
                //         mt_accumulatorGlobal= Ads[i];
                //         mt_accumulatorGlobal.inverse();
                //         const Eigen::Matrix<T, 6, 6> &invAd= mt_accumulatorGlobal.getMatrix();
                //         Ads[i].applyInverse(lbSE3[i].getVelocity(), mt_tempVelocity.getVelocity());
                //         mt_tempVelocity.calcMatrix();
                //         mt_6x6_1.noalias()= mt_sI.calculateSpatialInertia() * mt_tempVelocity.getMatrix();
                //         mt_6x6_2.noalias()= mt_6x6_1 - mt_6x6_1.transpose();
                //         mt_6x6_2*= invAd;
                //         mt_6x6_1.noalias()= invAd.transpose() * mt_6x6_2;
                //         mt_sI.applyInverseXIX(Ads[i]);
                //         mt_6xJ.noalias()= mt_6x6_1 * jacobians[i];
                //         mt_6xJ.noalias()+= mt_sI.calculateSpatialInertia() * jacobianDots[i];

                //         const_cast<Eigen::MatrixBase<Derived> &>(C).noalias()+= jacobians[i].transpose() * mt_6xJ;
                //         i++;
                //     }
                // } else
                if constexpr(OF == Frame::BODY) {
                    if(!link.isRoot() && !ARDL_visit(link.getParentJoint(), isFixed())) {
                        lbSE3[i].applyTranspose(link.getSI(), mt_6x6_1);
                        mt_6x6_2.setZero();
                        mt_6x6_2.noalias()= mt_6x6_1.transpose() - mt_6x6_1;
                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).setZero();
                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()=
                            mt_6x6_2 * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()+=
                            link.getSI().calculateSpatialInertia() *
                            jacobianDots[i].template block<6, -1>(0, 0, 6, i + 1);

                        const_cast<Eigen::MatrixBase<Derived> &>(C)
                            .template block<-1, -1>(0, 0, i + 1, i + 1)
                            .noalias()+= jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                                         mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);
                        i++;
                    }
                }
            }
        }

        /**
         * @brief Calculate Coriolis and centrifugal matrix
         *
         * @tparam Derived Subtype of output matrix
         * @param jacobians Aligned Vector of jacobians
         * @param jacobianDots Aligned vector of the derivative of the jacobians
         * @param lbSE3 Lie Bracket for each joint
         * @param C Output Coriolis and centrifugal matrix
         */
        template<Frame OF= Frame::BODY, typename Derived>
        void calcCoriolisMatrixOptim(aligned_vector<Jacobian<T>> &jacobians, aligned_vector<Jacobian<T>> &jacobianDots,
                                     aligned_vector<Motion<T>> &lbSE3, aligned_vector<Pose<T>> &Ads,
                                     Eigen::MatrixBase<Derived> const &C) {
            const_cast<Eigen::MatrixBase<Derived> &>(C).setZero();
            for(size_t i= 0; i < m_chain->getMoveableLinks().size() - 1; i++) {
                if constexpr(OF == Frame::BODY) {
                    lbSE3[i].applyTranspose(m_chain->getMoveableLinks()[i + 1]->getSI(), mt_6x6_1);
                    mt_6x6_2.setZero();
                    mt_6x6_2.noalias()= mt_6x6_1.transpose() - mt_6x6_1;
                    mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).setZero();
                    mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()=
                        mt_6x6_2 * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);

                    mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()+=
                        m_chain->getMoveableLinks()[i + 1]->getSI().calculateSpatialInertia() *
                        jacobianDots[i].template block<6, -1>(0, 0, 6, i + 1);

                    const_cast<Eigen::MatrixBase<Derived> &>(C).template block<-1, -1>(0, 0, i + 1, i + 1).noalias()+=
                        jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);
                }
            }
        }

        /**
         * @brief Calculate Gravity vector
         *
         * @tparam Derived Subtype of gravity vector
         * @param jacobians aligned vector of jacobians
         * @param Ads aligned vector of Adjoint transforms
         * @param G Output gravity vector
         */
        template<Frame OF= Frame::BODY, typename Derived>
        void calcGravityVector(aligned_vector<Jacobian<T>> &jacobians, aligned_vector<Pose<T>> &Ads,
                               Eigen::MatrixBase<Derived> const &G) {
            const_cast<Eigen::MatrixBase<Derived> &>(G).setZero();
            size_t i= 0;
            if constexpr(OF == Frame::BODY) { Ads[0].apply(m_gravity, mt_6x6_1.col(1)); }
            for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                    mt_sI= link->getSI();
                    if constexpr(OF == Frame::SPATIAL) {
                        mt_sI.applyInverseXIX(Ads[i]);
                        mt_gravity= m_gravity;
                        mt_sI.apply(mt_gravity, mt_6x6_1.col(0));
                    } else if constexpr(OF == Frame::BODY) {
                        Ads[i + 1].applyInverse(mt_6x6_1.col(1), mt_gravity);
                        mt_sI.apply(mt_gravity, mt_6x6_1.col(0));
                    }
                    const_cast<MatrixBase<Derived> &>(G).head(i + 1).noalias()+=
                        jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_gravity;
                    i++;
                }
            }
        }

        /**
         * @brief Calculate Gravity vector
         *
         * @tparam Derived Subtype of gravity vector
         * @param jacobians aligned vector of jacobians
         * @param Ads aligned vector of Adjoint transforms
         * @param G Output gravity vector
         */
        template<Frame OF= Frame::BODY, typename Derived>
        void calcGravityVectorOptim(aligned_vector<Jacobian<T>> &jacobians, aligned_vector<Pose<T>> &Ads,
                                    Eigen::MatrixBase<Derived> const &G) {
            const_cast<Eigen::MatrixBase<Derived> &>(G).setZero();
            if constexpr(OF == Frame::BODY) { Ads[0].apply(m_gravity, mt_6x6_1.col(1)); }
            for(size_t i= 0; i < m_chain->getMoveableLinks().size() - 1; i++) {
                mt_sI= m_chain->getMoveableLinksRef()[i + 1]->getSI();
                if constexpr(OF == Frame::SPATIAL) {
                    mt_sI.applyInverseXIX(Ads[i + 1]);
                    mt_gravity= m_gravity;
                    mt_sI.apply(mt_gravity, mt_6x6_1.col(0));
                } else if constexpr(OF == Frame::BODY) {
                    Ads[i + 1].applyInverse(mt_6x6_1.col(1), mt_gravity);
                    mt_sI.apply(mt_gravity, mt_6x6_1.col(0));
                }
                const_cast<MatrixBase<Derived> &>(G).head(i + 1).noalias()+=
                    jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_gravity;
                // i++;
            }
        }

        template<Frame OF= Frame::BODY, typename RD, typename JD>
        void calcRegressor(const Eigen::MatrixBase<JD> &qdd, const aligned_vector<Pose<T>> &Ads,
                           aligned_vector<Motion<T>> &adjs, const aligned_vector<Jacobian<T>> &jacs,
                           const aligned_vector<Jacobian<T>> &jDs, Eigen::MatrixBase<RD> const &out) {
            static_assert(Frame::BODY == OF, "BODY Frame is currently implemented");
            Matrix<T, 6, 1> mt_gravity;
            Motion<T> mt_bodyVelocity;
            LinkRegressor<T> mt_momentumRegressor, mt_momentumRegressor2;
            mt_momentumRegressor.setZero();
            mt_momentumRegressor2.setZero();
            Eigen::Matrix<T, 6, 2> mt_6x6_1;
            VectorX<T> qd= m_chain->getQd();
            Ads[0].apply(m_gravity, mt_gravity);
            for(size_t i= 0; i < m_chain->getNumOfJoints(); i++) {
                Ads[i + 1].applyInverse(mt_gravity, mt_bodyVelocity.getVelocity());
                mt_bodyVelocity+= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qdd.template head(i + 1);
                mt_6x6_1.col(0).noalias()= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qd.template head(i + 1);
                adjs[i].apply(mt_6x6_1.col(0), mt_6x6_1.col(1), mt_6x6_1.col(1));
                mt_bodyVelocity+= mt_6x6_1.col(1);
                mt_bodyVelocity+= jDs[i].template block<6, -1>(0, 0, 6, i + 1) * qd.template head(i + 1);
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                momentumRegressor(mt_6x6_1.col(0), mt_momentumRegressor2);
                adjs[i].calcMatrix();
                mt_momentumRegressor.noalias()-= adjs[i].getMatrix().transpose() * mt_momentumRegressor2;
                const_cast<Eigen::MatrixBase<RD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10).noalias()=
                    jacs[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
            }
        }

        template<Frame OF= Frame::BODY, typename RD, typename JD, typename JD2>
        void calcSlotineLiRegressor(const Eigen::MatrixBase<JD> &qd, const Eigen::MatrixBase<JD2> &qdd,
                                    const aligned_vector<Pose<T>> &Ads, aligned_vector<Motion<T>> &adjs,
                                    const aligned_vector<Jacobian<T>> &jacs, const aligned_vector<Jacobian<T>> &jDs,
                                    Eigen::MatrixBase<RD> const &out) {
            static_assert(Frame::BODY == OF, "BODY Frame is currently implemented");
            Eigen::Matrix<T, 6, 1> mt_gravity;
            Motion<T> mt_bodyVelocity;
            Eigen::Matrix<T, 6, 10> mt_momentumRegressor, mt_momentumRegressor2, mt_momentumRegressor3;
            mt_momentumRegressor.setZero();
            mt_momentumRegressor2.setZero();
            Eigen::Matrix<T, 6, 2> mt_6x6_1;
            Ads[0].apply(m_gravity, mt_gravity);
            for(size_t i= 0; i < m_chain->getNumOfJoints(); i++) {
                Ads[i + 1].applyInverse(mt_gravity, mt_bodyVelocity.getVelocity());
                mt_bodyVelocity+= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qdd.head(i + 1);
                mt_6x6_1.col(0).noalias()= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                adjs[i].apply(mt_6x6_1.col(0), mt_6x6_1.col(1), mt_6x6_1.col(1));
                mt_bodyVelocity+= mt_6x6_1.col(1);
                mt_bodyVelocity+= jDs[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                momentumRegressor(mt_6x6_1.col(0), mt_momentumRegressor2);
                // adjs[i].applyTransposeTo(mt_momentumRegressor2, mt_momentumRegressor3);
                adjs[i].calcMatrix();
                mt_momentumRegressor3= adjs[i].getMatrix().transpose() * mt_momentumRegressor2;
                mt_momentumRegressor.noalias()-= mt_momentumRegressor3;
                const_cast<Eigen::MatrixBase<RD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10).noalias()=
                    jacs[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
            }
        }

        template<typename RegressorD, typename JointD>
        void initFilteredSlotineLiRegressor(const aligned_vector<Jacobian<T>> &jacobians,
                                            const aligned_vector<Jacobian<T>> &jacobianDots,
                                            const aligned_vector<Motion<T>> &lbSE3,
                                            const aligned_vector<Pose<T>> &Ads,
                                            const Eigen::MatrixBase<JointD> &qd, DiscreteLowPassFilter<T> &filter,
                                            Eigen::MatrixBase<RegressorD> const &out) {
            const_cast<Eigen::MatrixBase<RegressorD> &>(out).setZero();

            Ads[0].apply(m_gravity, mt_gravity);
            for(int i= 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                Ads[i + 1].applyInverse(mt_gravity, mt_6x6_1.col(0));

                mt_bodyVelocity= (filter.getBeta() * jacobians[i] * qd);
                mt_bodyVelocity-= mt_6x6_1.col(0);

                mt_6x6_1.col(0).noalias()= jacobians[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                lbSE3[i].apply(mt_6x6_1.col(0), mt_6x6_1.col(1));
                mt_bodyVelocity-= mt_6x6_1.col(1);

                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                momentumRegressor(mt_6x6_1.col(0), mt_momentumRegressor2);

                mt_momentumRegressor.noalias()+= lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2;

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)=
                    jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)+=
                    jacobianDots[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor2;
            }
            filter.setBuffer(out);
            const_cast<Eigen::MatrixBase<RegressorD> &>(out).setZero();
            for(int i= 0; i < m_chain->getNumOfJoints(); i++) {
                // mt_bodyVelocity.getVelocity().setZero();
                mt_bodyVelocity= filter.getBeta() * jacobians[i] * qd;
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)=
                    jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
            }
            const_cast<Eigen::MatrixBase<RegressorD> &>(out)-= (filter.getResult());
        }

        template<typename RegressorD, typename JointD>
        void computeFilteredSlotineLiRegressor(const aligned_vector<Jacobian<T>> &jacobians,
                                               const aligned_vector<Jacobian<T>> &jacobianDots,
                                               const aligned_vector<Motion<T>> &lbSE3,
                                               const aligned_vector<Pose<T>> &Ads,
                                               const Eigen::MatrixBase<JointD> &qd, DiscreteLowPassFilter<T> &filter,
                                               Eigen::MatrixBase<RegressorD> const &out) {
            const_cast<Eigen::MatrixBase<RegressorD> &>(out).setZero();

            Ads[0].apply(m_gravity, mt_gravity);
            for(int i= 0; i < m_chain->getNumOfJoints(); i++) {
                // mt_bodyVelocity.getVelocity().setZero();
                Ads[i + 1].applyInverse(mt_gravity, mt_6x6_1.col(0));

                mt_bodyVelocity= (filter.getBeta() * jacobians[i] * qd);
                mt_bodyVelocity-= mt_6x6_1.col(0);

                mt_6x6_1.col(0).noalias()= jacobians[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                lbSE3[i].apply(mt_6x6_1.col(0), mt_6x6_1.col(1));
                mt_bodyVelocity-= mt_6x6_1.col(1);

                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                momentumRegressor(mt_6x6_1.col(0), mt_momentumRegressor2);

                mt_momentumRegressor.noalias()+= lbSE3[i].getMatrix().transpose() * mt_momentumRegressor2;

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)=
                    jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)+=
                    jacobianDots[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor2;
            }
            filter.compute(out);
            const_cast<Eigen::MatrixBase<RegressorD> &>(out).setZero();
            for(int i= 0; i < m_chain->getNumOfJoints(); i++) {
                // mt_bodyVelocity.getVelocity().setZero();
                mt_bodyVelocity= filter.getBeta() * jacobians[i] * qd;
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                const_cast<Eigen::MatrixBase<RegressorD> &>(out).template block<-1, 10>(0, 10 * i, i + 1, 10)=
                    jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
            }
            const_cast<Eigen::MatrixBase<RegressorD> &>(out)-= (filter.getResult());
        }

        void calcBaseProjection(size_t random_samples, T qddLimit= 1, T threshold= T(1e-8)) {
            // Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressors(random_samples * m_chain->getNumOfJoints(),
            //                                                             10 * m_chain->getNumOfJoints());
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R(10 * m_chain->getNumOfJoints(),
                                                               10 * m_chain->getNumOfJoints());
            ForwardKinematics<T> fk(m_chain);
            aligned_vector<Pose<T>> Ads(m_chain->getNumOfJoints() + 1);
            aligned_vector<Motion<T>> lbSE3(m_chain->getNumOfJoints());
            aligned_vector<Jacobian<T>> jacobians, jacobianDots;

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> regressors(m_chain->getNumOfJoints(),
                                                                        10 * m_chain->getNumOfJoints());

            ARDL::Util::init(jacobians, m_chain->getNumOfJoints());
            ARDL::Util::init(jacobianDots, m_chain->getNumOfJoints());

            VectorX<T> q, qd, qdd;
            q.resize(m_chain->getNumOfJoints());
            qd.resize(m_chain->getNumOfJoints());
            qdd.resize(m_chain->getNumOfJoints());
            R.setZero();

            regressors.setZero();
            SobolData *sobolGen= sobol_create(m_chain->getNumOfJoints() * 3);
            std::vector<std::pair<T, T>> limits(m_chain->getNumOfJoints());
            m_chain->getJointLimits(limits);
            ARDL::VectorX<T> qdLimits(m_chain->getNumOfJoints());
            m_chain->getJointVelocityLimits(qdLimits);
            double *sobolRandom= (double *) malloc(m_chain->getNumOfJoints() * 3 * sizeof(double));
            double *lb= (double *) malloc(m_chain->getNumOfJoints() * 3 * sizeof(double));
            double* ub = (double*)malloc(m_chain->getNumOfJoints()*3*sizeof(double));
            for(size_t i = 0; i<m_chain->getNumOfJoints();i++){
                lb[i] = limits[i].first;
                ub[i] = limits[i].second;
                lb[i+m_chain->getNumOfJoints()] = -qdLimits(i);
                ub[i+m_chain->getNumOfJoints()] = qdLimits(i);
                lb[i+2*m_chain->getNumOfJoints()] = -qddLimit;
                ub[i+2*m_chain->getNumOfJoints()] = qddLimit;
            }

            // CREATE RANDOM REGRESSOR STACK
            for(size_t k= 0; k < random_samples; k++) {
                sobol_next(sobolGen, sobolRandom, lb,ub);
                for(size_t i = 0;i<m_chain->getNumOfJoints();i++){
                    q(i) = (T) sobolRandom[i];
                    qd(i) = (T) sobolRandom[i+m_chain->getNumOfJoints()];
                    qdd(i) = (T) sobolRandom[i+2*m_chain->getNumOfJoints()];
                }
                // m_chain->random();
                // qd= m_chain->getQd();
                // qdd.setRandom();
                // qdd*= qddLimit;
                m_chain->updateChain(q,qd);
                m_chain->updateMatricesOptim();
                fk.getBodyAdjointsOptim(Ads);

                fk.template getJacobians<ARDL::Frame::BODY>(Ads, jacobians);
                fk.getLieBrackets(lbSE3, jacobians);
                fk.template getJacobianDots<ARDL::Frame::BODY>(Ads, lbSE3, jacobians, jacobianDots);
                this->calcSlotineLiRegressor<ARDL::Frame::BODY>(qd, qdd, Ads, lbSE3, jacobians, jacobianDots,
                                                                regressors);

                R+= (regressors.transpose() * regressors);// / random_samples;
            }
            free(sobolRandom);
            free(lb);
            free(ub);
            sobol_destroy(sobolGen);
            Eigen::ColPivHouseholderQR<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> qr(R);
            qr.setThreshold(threshold);
            qr.compute(R);

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Q(m_chain->getNumOfJoints() * 10,
                                                               m_chain->getNumOfJoints() * 10);
            Q.setZero();
            Q= qr.householderQ();

            Eigen::Matrix<T, -1, -1> RQ(m_chain->getNumOfJoints() * 10, m_chain->getNumOfJoints() * 10);
            RQ.setZero();
            RQ= qr.matrixR();

            Eigen::Matrix<T, -1, -1> PQ(m_chain->getNumOfJoints() * 10, m_chain->getNumOfJoints() * 10);
            PQ.setZero();
            PQ= qr.colsPermutation();

            m_baseParamNo= qr.rank();
            // Pb
            m_projRegressor= PQ.block(0, 0, PQ.rows(), m_baseParamNo);

            // R1^{-1}*R2
            m_projParameters= RQ.block(0, 0, m_baseParamNo, m_baseParamNo).inverse() *
                              RQ.block(0, m_baseParamNo, m_baseParamNo, RQ.cols() - m_baseParamNo);

            m_projParameters=
                (m_projParameters.array().abs() > threshold).matrix().template cast<T>().cwiseProduct(m_projParameters);

            ARDL::MatrixX<T> K=
                m_projRegressor.transpose() +
                m_projParameters * PQ.block(0, m_baseParamNo, PQ.rows(), PQ.cols() - m_baseParamNo).transpose();
            m_projParameters= K;
            // m_projRegressor = K.transpose();
            // m_projRegressor= m_projParameters.transpose() * (m_projParameters *
            // m_projParameters.transpose()).inverse();
            // ARDL::MatrixX<T> Wr(m_baseParamNo, m_baseParamNo);
            // ARDL::MatrixX<T> We(m_baseParamNo, RQ.rows()-m_baseParamNo);
            // Wr= Q.block(0, 0, Q.rows(), m_baseParamNo) * RQ.block(0, 0, m_baseParamNo, m_baseParamNo);
            // We =
            m_regressorProjected.resize(m_chain->getNumOfJoints(), m_baseParamNo);
        }

        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &getRegressorProjector() { return m_projRegressor; }

        void setRegressorProjector(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& in) { m_projRegressor = in; m_baseParamNo = in.cols(); }
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &getParameterProjector() { return m_projParameters; }
        const size_t &getNumOfBaseParams() { return m_baseParamNo; }

        const ARDL::MatrixX<T> &projectRegressor(const ARDL::Regressor<T> &regress) {
            m_regressorProjected= regress * m_projRegressor;
            return m_regressorProjected;
        }

        template<Frame OF= Frame::BODY>
        void calcJointInertiaMatrixDq(const aligned_vector<Pose<T>> Ads,
                                      const aligned_vector<Jacobian<T>> &jacobians,
                                      const aligned_vector<aligned_vector<Jacobian<T>>> &jacobiansDq,
                                      aligned_vector<MatrixX<T>> &M) {
            for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                static_assert(OF == Frame::BODY, "Only BODY Frame has been implemented currently");
                size_t i= 0;
                M[i1].setZero();
                for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                    if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                        mt_sI= link->getSI();
                        mt_sI.applyOut(jacobians[i].template block<6, Dynamic>(0, 0, 6, i + 1),
                                       mt_6xJ.template block<6, Dynamic>(0, 0, 6, i + 1));
                        M[i1].template block<Dynamic, Dynamic>(0, 0, i + 1, i + 1).noalias()+=
                            jacobiansDq[i1][i].template block<6, Dynamic>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, Dynamic>(0, 0, 6, i + 1);
                        mt_sI.applyOut(jacobiansDq[i1][i].template block<6, Dynamic>(0, 0, 6, i + 1),
                                       mt_6xJ.template block<6, Dynamic>(0, 0, 6, i + 1));
                        M[i1].template block<Dynamic, Dynamic>(0, 0, i + 1, i + 1).noalias()+=
                            jacobians[i].template block<6, Dynamic>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, Dynamic>(0, 0, 6, i + 1);
                        i++;
                    }
                }
            }
        }
        // TODO optimize
        template<Frame OF= Frame::BODY>
        void calcCoriolisMatrixDq(aligned_vector<Motion<T>> &lbSE3, const aligned_vector<Jacobian<T>> &jacobians,
                                  aligned_vector<Jacobian<T>> &jacobianDots,
                                  const aligned_vector<aligned_vector<Jacobian<T>>> &jacobiansDq,
                                  const aligned_vector<aligned_vector<Jacobian<T>>> &jacobianDotsDq,
                                  aligned_vector<Eigen::Matrix<T, -1, -1>> &C) {
            for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                size_t i= 0;
                C[i1].setZero();
                for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                    if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                        lbSE3[i].calcMatrix();
                        mt_6x6_1= lbSE3[i].getMatrix();
                        link->getSI().apply(mt_6x6_1, mt_6x6_2);
                        lbSE3[i].applyTranspose(link->getSI(), mt_6x6_2);

                        mt_6x6_1-= mt_6x6_2;

                        link->getSI().applyOut(jacobianDots[i].template block<6, -1>(0, 0, 6, i + 1),
                                               mt_6xJ.template block<6, -1>(0, 0, 6, i + 1));

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()+=
                            mt_6x6_1 * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);
                        C[i1].template block<-1, -1>(0, 0, i + 1, i + 1).noalias()+=
                            jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);

                        link->getSI().applyOut(jacobianDotsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1),
                                               mt_6xJ.template block<6, -1>(0, 0, 6, i + 1));

                        C[i1].template block<-1, -1>(0, 0, i + 1, i + 1).noalias()+=
                            jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1)=
                            mt_6x6_1 * jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1);

                        mt_bodyVelocity=
                            jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) * m_chain->getQd().head(i + 1);
                        mt_bodyVelocity.calcMatrix();
                        mt_6x6_1= mt_bodyVelocity.getMatrix();
                        link->getSI().apply(mt_6x6_1, mt_6x6_2);
                        mt_bodyVelocity.applyTranspose(link->getSI(), mt_6x6_2);

                        mt_6x6_1-= mt_6x6_2;

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()+=
                            mt_6x6_1 * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);
                        C[i1].template block<-1, -1>(0, 0, i + 1, i + 1).noalias()+=
                            jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);

                        i++;
                    }
                }
            }
        }

        template<Frame OF= Frame::BODY, int j>
        void calcCoriolisMatrixDqd(const aligned_vector<Jacobian<T>> &jacobians,
                                   const aligned_vector<Jacobian<T>> &jacobianDots,
                                   const aligned_vector<aligned_vector<Jacobian<T>>> &jacobiansDq,
                                   aligned_vector<Eigen::Matrix<T, j, j>> &C) {
            for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                size_t i= 0;
                C[i1].setZero();
                for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                    if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                        mt_bodyVelocity.setVelocity(jacobians[i].col(i1));
                        mt_bodyVelocity.calcMatrix();
                        mt_6x6_1= mt_bodyVelocity.getMatrix();
                        link->getSI().apply(mt_6x6_1, mt_6x6_2);
                        mt_bodyVelocity.applyTranspose(link->getSI(), mt_6x6_2);

                        mt_6x6_1-= mt_6x6_2;

                        link->getSI().applyOut(jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1),
                                               mt_6xJ.template block<6, -1>(0, 0, 6, i + 1));

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()+=
                            mt_6x6_1 * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);

                        C[i1].template block<-1, -1>(0, 0, i + 1, i + 1)+=
                            jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                            mt_6xJ.template block<6, -1>(0, 0, 6, i + 1);
                        i++;
                    }
                }
            }
        }

        template<Frame OF= Frame::BODY, int j>
        void calcGravityVectorDq(const aligned_vector<Pose<T>> Ads, const aligned_vector<Jacobian<T>> &jacobians,
                                 const aligned_vector<aligned_vector<Jacobian<T>>> &jacobiansDq,
                                 aligned_vector<Eigen::Matrix<T, j, 1>> &G) {
            for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) { G[i1].setZero(); }
            size_t i= 0;
            if constexpr(OF == Frame::BODY) { Ads[0].apply(m_gravity, mt_6x6_1.col(1)); }
            for(std::shared_ptr<Link<T>> &link: m_chain->getLinksRef()) {
                if(!link->isRoot() && !ARDL_visit_ptr(link->getParentJoint(), isFixed())) {
                    mt_sI= link->getSI();
                    if constexpr(OF == Frame::BODY) { Ads[i + 1].applyInverse(mt_6x6_1.col(1), mt_gravity); }
                    for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                        mt_bodyVelocity.setVelocity(jacobians[i].col(i1));
                        mt_6x6_2= mt_bodyVelocity.getMatrix();
                        link->getSI().apply(mt_6x6_2, mt_6x6_3);

                        link->getSI().applyOut(jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1),
                                               mt_6xJ.template block<6, -1>(0, 0, 6, i + 1));

                        mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).noalias()-=
                            mt_6x6_2.transpose() * jacobians[i].template block<6, -1>(0, 0, 6, i + 1);
                        G[i1].head(i + 1).noalias()+=
                            mt_6xJ.template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_gravity;
                    }
                    i++;
                }
            }
        }

        template<Frame OF= Frame::BODY, typename JointD>
        void calcSlotineLiRegressorDq(const aligned_vector<Jacobian<T>> &jacs,
                                      const aligned_vector<Jacobian<T>> &jacDots,
                                      const aligned_vector<aligned_vector<Jacobian<T>>> &jacsDq,
                                      const aligned_vector<aligned_vector<Jacobian<T>>> &jacDotsDq,
                                      aligned_vector<Motion<T>> &adjs, const aligned_vector<Pose<T>> &Ads,
                                      const Eigen::MatrixBase<JointD> &qd, const Eigen::MatrixBase<JointD> &qdd,
                                      aligned_vector<Regressor<T>> &regressorsDq) {
            static_assert(Frame::BODY == OF, "BODY Frame is currently implemented");

            Ads[0].apply(m_gravity, mt_gravity);
            for(size_t i= 0; i < m_chain->getNumOfJoints(); i++) {
                mt_bodyVelocity.getVelocity().setZero();
                Ads[i + 1].applyInverse(mt_gravity, mt_bodyVelocity.getVelocity());
                // J * qdd_r + Ads^-1 grav
                mt_bodyVelocity+= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qdd.head(i + 1);
                // J * q_r
                mt_6x6_1.col(0).noalias()= jacs[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                // adj * J * q_r
                adjs[i].apply(mt_6x6_1.col(0), mt_6x6_1.col(1));
                // J qdd_r + adj J q_r + Ads^-1 grav
                mt_bodyVelocity+= mt_6x6_1.col(1);
                // J qdd_r + adj J q_r + Jd qd_r + Ads^-1 grav
                mt_bodyVelocity+= jacDots[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                // A(J qdd_r + adj J q_r + Jd qd + Ads^-1 grav)
                momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor3);
                // A(J q_r)
                momentumRegressor(mt_6x6_1.col(0), mt_momentumRegressor2);
                adjs[i].calcMatrix();
                // A(J qdd_r + adj J q_r + Jd qd + Ads^-1 grav) - adjsT A(J q_r)
                mt_momentumRegressor3.noalias()-= adjs[i].getMatrix().transpose() * mt_momentumRegressor2;
                for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                    // J/dq^T (A(J qdd_r + adj J q_r + Jd qd + Ads^-1 grav) - adjsT A(J q_r))
                    regressorsDq[i1].template block<-1, 10>(0, 10 * i, i + 1, 10).noalias()=
                        jacsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor3;

                    Ads[i + 1].applyInverse(mt_gravity, mt_bodyVelocity.getVelocity());
                    mt_tempVelocity= jacs[i].col(i1);
                    //-adj_jk * Ad^-1 * grav
                    mt_tempVelocity.apply(mt_bodyVelocity.getVelocity(), mt_6x6_1.col(5), mt_6x6_1.col(5));
                    mt_bodyVelocity= -mt_6x6_1.col(5);

                    // J/dq qdd-adj * Ad^-1 * grav
                    mt_bodyVelocity+= jacsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) * qdd.head(i + 1);
                    // Jd/dq qd+J/dq qdd-adj * Ad^-1 * grav
                    mt_bodyVelocity+= jacDotsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);

                    // J/dq qd_r
                    mt_6x6_1.col(2)= jacsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                    adjs[i].apply(mt_6x6_1.col(2), mt_6x6_1.col(3));
                    // Jd/dq qd+J/dq qdd-adj * Ad^-1 * grav + adjs J/dq qd
                    mt_bodyVelocity+= mt_6x6_1.col(3);

                    // J/dq qd
                    mt_tempVelocity.setVelocity(jacsDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) *
                                                m_chain->getQd().head(i + 1));
                    // adj_(J/dq qd) J qd
                    mt_tempVelocity.apply(mt_6x6_1.col(0), mt_6x6_1.col(4), mt_6x6_1.col(4));
                    // Jd/dq qd+J/dq qdd-adj * Ad^-1 * grav + adjs J/dq qd + adj_(J/dq qd) J qd
                    mt_bodyVelocity+= mt_6x6_1.col(4);

                    // A(Jd/dq qd+J/dq qdd-adj * Ad^-1 * grav + adjs J/dq qd + adj_(J/dq qd) J qd)
                    momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);
                    regressorsDq[i1].template block<-1, 10>(0, 10 * i, i + 1, 10).noalias()+=
                        jacs[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;

                    ////////////////////////////////////////////////////////////

                    momentumRegressor(mt_6x6_1.col(2), mt_momentumRegressor);
                    // TODO optimize
                    mt_momentumRegressor= adjs[i].getMatrix().transpose() * mt_momentumRegressor;

                    mt_tempVelocity.calcMatrix();
                    mt_momentumRegressor.noalias()+= mt_tempVelocity.getMatrix().transpose() * mt_momentumRegressor2;
                    regressorsDq[i1].template block<-1, 10>(0, 10 * i, i + 1, 10).noalias()-=
                        jacs[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
                }
            }
        }

        template<typename JointD>
        void calcSlotineLiRegressorDqd(const aligned_vector<Jacobian<T>> &jacobians,
                                       const aligned_vector<Jacobian<T>> &jacobianDots,
                                       const aligned_vector<aligned_vector<Jacobian<T>>> &jacobiansDq,
                                       const aligned_vector<Motion<T>> &lbSE3,
                                       const Eigen::MatrixBase<JointD> &qd,
                                       aligned_vector<Regressor<T>> &regressorsDq) {
            for(size_t i= 0; i < m_chain->getNumOfJoints(); i++) {
                // Jq
                mt_6x6_1.col(3).noalias()= jacobians[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);

                // A(Jq)
                momentumRegressor(jacobians[i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1),
                                  mt_momentumRegressor2);

                for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                    // adj(J^h_k)
                    mt_tempVelocity.setVelocity(jacobians[i].col(i1));
                    // adj(J^h_k) Jq
                    mt_tempVelocity.apply(mt_6x6_1.col(3), mt_bodyVelocity.getVelocity(), mt_6x6_1.col(0));

                    // adj_0,k J^h_k
                    lbSE3[i].apply(jacobians[i].col(i1), mt_6x6_1.col(1));

                    // adj(J^h_k) Jq + adj_0,k J^h_k
                    mt_bodyVelocity+= mt_6x6_1.col(1);
                    // adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd
                    mt_bodyVelocity+= jacobiansDq[i1][i].template block<6, -1>(0, 0, 6, i + 1) * qd.head(i + 1);
                    // adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd+ J^h_k
                    mt_bodyVelocity+= jacobianDots[i].col(i1);
                    // A(adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd+ J^h_k)
                    momentumRegressor(mt_bodyVelocity.getVelocity(), mt_momentumRegressor);

                    // TODO optimize
                    // A(adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd+ J^h_k) - adj(J^h_k)^T A(Jq)
                    mt_momentumRegressor.noalias()-= mt_tempVelocity.getMatrix().transpose() * mt_momentumRegressor2;

                    // J^T A(adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd+ J^h_k) - adj(J^h_k)^T A(Jq)
                    regressorsDq[i1].template block<Eigen::Dynamic, 10>(0, 10 * i, i + 1, 10).noalias()=
                        jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * (mt_momentumRegressor);

                    // A(J^h_k)
                    momentumRegressor(jacobians[i].col(i1), mt_momentumRegressor);
                    //(J^T A(adj(J^h_k) Jq + adj_0,k J^h_k + Jdq[i1][i] qd+ J^h_k) - adj(J^h_k)^T A(Jq)) - J^{T}
                    // adj_{0,k} A(J^h_k)
                    regressorsDq[i1].template block<Eigen::Dynamic, 10>(0, 10 * i, i + 1, 10).noalias()-=
                        jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() *
                        lbSE3[i].getMatrix().transpose() * mt_momentumRegressor;
                }
            }
        }
        void calcSlotineLiRegressorDqdd(const aligned_vector<Jacobian<T>> &jacobians,
                                        aligned_vector<Regressor<T>> &regressorsDq) {
            for(size_t i1= 0; i1 < m_chain->getNumOfJoints(); i1++) {
                regressorsDq[i1].setZero();
                for(size_t i= 0; i < m_chain->getNumOfJoints(); i++) {
                    momentumRegressor(jacobians[i].col(i1), mt_momentumRegressor);
                    regressorsDq[i1].template block<Eigen::Dynamic, 10>(0, 10 * i, i + 1, 10).noalias()=
                        jacobians[i].template block<6, -1>(0, 0, 6, i + 1).transpose() * mt_momentumRegressor;
                }
            }
        }
    }; // namespace ARDL
} // namespace ARDL

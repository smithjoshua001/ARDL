#pragma once

#include "ARDL/Dynamics/Dynamics.hpp"
#include "IGMMModel.hpp"
#include "ARDL/Semi-Parametric/SPModel.hpp"
#include "ARDL/Util/MatrixInitializer.hpp"

using namespace ARDL;
using namespace ARDL::Model;
using namespace ARDL::Util;
using namespace ARDL::Math;
using namespace ARDL::NP;
namespace ARDL {
    namespace SP {
        template<typename T>
        class IGMMSPModel: public SPModel<T,IGMMSPModel<T>, IGMMModel<T>> {
             private:
            aligned_vector<std::pair<MVN<T>, T>> &mvns;

            size_t dof;

            std::shared_ptr<Chain<T>> c;
            std::shared_ptr<ForwardKinematics<T>> fk;
            std::shared_ptr<Dynamics<T>> dyn;
            aligned_vector<Pose<T>> ads;
            aligned_vector<Motion<T>> adjs;
            aligned_vector<Jacobian<T>> jacs, jacDots;
            aligned_vector<aligned_vector<Jacobian<T>>> jacsDq, jacDotsDq;
            aligned_vector<Regressor<T>> regressorDq, regressorDqd, regressorDqdd;
            Eigen::JacobiSVD<MatrixX<T>> svd;

            VectorX<T> xC, pi, pid, R;

            MatrixX<T> covDot, cov, Yb,  Wb, Rho, K;
            Regressor<T> Y,W;

            T lambda0, k0, lambda;

            DiscreteLowPassFilter<T> filt;
            DiscreteLowPassFilter<T> *tau_filt;

            bool init= false;

            VectorX<T> tauNP, tauP, svdSingular;

             public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            IGMMSPModel(std::string urdfModel, double initialRhoInverse, double RWeightTorque, double lambdaScale, double maxRhoEigenvalue, double k1,
                        DiscreteLowPassFilter<double> filt, size_t dims, double metaChi, Eigen::VectorXd initCov)
                : SPModel<T,IGMMSPModel<T>, IGMMModel<T>>(std::move(IGMMModel<double>(dims * 4, dims, metaChi, initCov))),
                  mvns(this->npModel.derived().getModels()), filt(filt) {
                c= std::shared_ptr<Chain<double>>(new Chain<double>(urdfModel));
                fk= std::shared_ptr<ForwardKinematics<double>>(new ForwardKinematics<double>(c));
                dyn= std::shared_ptr<Dynamics<double>>(new Dynamics<double>(c));
                dyn->calcBaseProjection(50000, 5);
                dof= dims;
                std::cout << "IGMMSP DIMS: " << dims << std::endl;
                ads.resize(dof + 1);
                adjs.resize(dof);
                ARDL::Util::init(jacs, dof);
                ARDL::Util::init(jacDots, dof);
                ARDL::Util::init(jacsDq, dof);
                ARDL::Util::init(jacDotsDq, dof);

                ARDL::Util::init(regressorDq, dof);
                ARDL::Util::init(regressorDqd, dof);
                ARDL::Util::init(regressorDqdd, dof);

                tauP.resize(dof);
                tauNP.resize(dof);

                xC.resize(dof * 4);
                covDot.resize(dof * 4, dof * 4);
                covDot.setIdentity();
                cov.resize(dof * 4, dof * 4);
                ARDL::Util::init(Y, dof);
                ARDL::Util::init(W, dof);
                Yb.resize(dof, dyn->getNumOfBaseParams());
                Yb.setZero();
                Wb.resize(dof, dyn->getNumOfBaseParams());
                Wb.setZero();

                pi.resize(dyn->getNumOfBaseParams());
                pi.setZero();
                pid.resize(dyn->getNumOfBaseParams());
                pid.setZero();

                R.resize(dof);
                R.setConstant(RWeightTorque);

                Rho.resize(dyn->getNumOfBaseParams(), dyn->getNumOfBaseParams());
                Rho.setZero();
                Rho.diagonal().setConstant(1 / initialRhoInverse);
                svd.compute(Rho, ComputeThinU | ComputeThinV);
                std::cout << (Rho.inverse().colwise().template lpNorm<1>().maxCoeff()) << std::endl;
                K.resize(dyn->getNumOfBaseParams(), dyn->getNumOfBaseParams());
                K.setZero();
                K.diagonal().setConstant(k1);

                svdSingular.resize(dyn->getNumOfBaseParams());

                lambda0= lambdaScale;
                this->k0= maxRhoEigenvalue;

                tau_filt= new DiscreteLowPassFilter<double>(filt.getCutoff(), filt.getSamplingRate(), dof, 1);
            }
            ~IGMMSPModel() { delete tau_filt; }
            template<typename muD, typename outD, typename qD, typename qdD, typename qd_rD, typename qdd_rD>
            void compute(const Eigen::MatrixBase<muD> &input, const Eigen::MatrixBase<qD> &q,
                         const Eigen::MatrixBase<qdD> &qd, const Eigen::MatrixBase<qd_rD> &qd_r,
                         const Eigen::MatrixBase<qdd_rD> &qdd_r, const Eigen::MatrixBase<outD> &output) {
                c->updateChain(q, qd);
                c->updateMatrices();
                fk->getBodyAdjoints(ads);
                fk->template getJacobians<ARDL::Frame::BODY>(ads, jacs);
                fk->getLieBrackets(adjs, jacs);
                fk->template getJacobianDots<ARDL::Frame::BODY>(ads, adjs, jacs, jacDots);
                Y.setZero();
                dyn->template calcSlotineLiRegressor<ARDL::Frame::BODY>(qd_r, qdd_r, ads, adjs, jacs, jacDots, Y);

                this->npModel.derived().compute(input.segment(0, dof * 3),
                                                const_cast<Eigen::MatrixBase<outD> &>(output));
                tauNP= output;
                Yb.noalias()= Y * dyn->getRegressorProjector();
                tauP= Yb * pi;
                const_cast<Eigen::MatrixBase<outD> &>(output).noalias()+= tauP;
            }
            template<typename D, typename sD>
            void learn(const Eigen::MatrixBase<D> &x, const Eigen::MatrixBase<sD> &s) {
                W.setZero();
                if(!init) {
                    dyn->initFilteredSlotineLiRegressor(jacs, jacDots, adjs, ads, x.segment(dof, dof), filt, W);
                    tau_filt->setBuffer(x.segment(dof * 3, dof));
                    init= true;
                } else {
                    dyn->computeFilteredSlotineLiRegressor(jacs, jacDots, adjs, ads, x.segment(dof, dof), filt, W);
                    tau_filt->compute(x.segment(dof * 3, dof));
                }
                Wb= W * dyn->getRegressorProjector();
                lambda= lambda0 * (1.0 - Rho.inverse().colwise().template lpNorm<1>().maxCoeff() / k0);
                dyn->template calcSlotineLiRegressor<ARDL::Frame::BODY>(x.segment(dof, dof), x.segment(dof * 2, dof),
                                                                        ads, adjs, jacs, jacDots, Y);
                Yb= Y * dyn->getRegressorProjector();
                if(Rho.inverse().colwise().template lpNorm<1>().maxCoeff() > K(0, 0)) {
                    Rho.noalias()+= Wb.transpose() * Wb * 0.001;
                } else {
                    Rho+= (-lambda * Rho + (Wb.transpose() * Wb)) * 0.001;
                }

                pid= 0.001 * Rho.inverse() *
                     (Yb.transpose() * s + (Wb.transpose() * R.asDiagonal() * (Wb * pi - tau_filt->getResult())));
                pi-= pid;

                xC= x;
                xC.segment(dof * 3, dof)-= Yb * pi;

                VectorX<T> &mu= this->npModel.derived().getMu();

                MatrixX<T> &M2= this->npModel.derived().getM2();
                c->updateChain(mu.head(dof), mu.segment(dof, dof));
                c->updateMatrices();
                fk->getBodyAdjoints(ads);
                fk->template getJacobians<ARDL::Frame::BODY>(ads, jacs);
                fk->getLieBrackets(adjs, jacs);
                fk->template getJacobianDots<ARDL::Frame::BODY>(ads, adjs, jacs, jacDots);
                fk->getJacobiansDq(ads, jacs, jacsDq);
                fk->getJacobianDotsDq(ads, adjs, jacDots, jacsDq, jacDotsDq);

                dyn->template calcSlotineLiRegressor<ARDL::Frame::BODY>(mu.segment(dof, dof), mu.segment(dof * 2, dof),
                                                                        ads, adjs, jacs, jacDots, Y);

                dyn->calcSlotineLiRegressorDq(jacs, jacDots, jacsDq, jacDotsDq, adjs, ads, mu.segment(dof, dof),
                                              mu.segment(dof * 2, dof), regressorDq);

                dyn->calcSlotineLiRegressorDqd(jacs, jacDots, jacsDq, adjs, mu.segment(dof, dof), regressorDqd);
                dyn->calcSlotineLiRegressorDqdd(jacs, regressorDqdd);
                Yb= Y * dyn->getRegressorProjector();
                mu.segment(dof * 3, dof)+= Yb * pid;
                for(size_t i0= 0; i0 < dof; i0++) {
                    Yb= regressorDq[i0] * dyn->getRegressorProjector();
                    covDot.block(dof * 3, i0, dof, 1)= Yb * pid;
                    Yb= regressorDqd[i0] * dyn->getRegressorProjector();
                    covDot.block(dof * 3, i0 + dof, dof, 1)= Yb * pid;
                    Yb= regressorDqdd[i0] * dyn->getRegressorProjector();
                    covDot.block(dof * 3, i0 + dof + dof, dof, 1)= Yb * pid;
                }
                cov= (M2*this->npModel.derived().getCount()).inverse();
                cov*= covDot.transpose();
                M2.noalias()= covDot * cov;
                cov= M2;
                M2= cov.inverse()/this->npModel.derived().getCount();

                for(size_t i= 0; i < this->npModel.derived().getNumberOfModels(); i++) {
                    // if(mvns[i].first.getVs()>56){
                    c->updateChain(mvns[i].first.getMu().head(dof), mvns[i].first.getMu().segment(dof, dof));
                    c->updateMatrices();
                    fk->getBodyAdjoints(ads);
                    fk->template getJacobians<ARDL::Frame::BODY>(ads, jacs);
                    fk->getLieBrackets(adjs, jacs);
                    fk->template getJacobianDots<ARDL::Frame::BODY>(ads, adjs, jacs, jacDots);
                    fk->getJacobiansDq(ads, jacs, jacsDq);
                    fk->getJacobianDotsDq(ads, adjs, jacDots, jacsDq, jacDotsDq);

                    dyn->template calcSlotineLiRegressor<ARDL::Frame::BODY>(mvns[i].first.getMu().segment(dof, dof),
                                                                            mvns[i].first.getMu().segment(dof * 2, dof),
                                                                            ads, adjs, jacs, jacDots, Y);

                    dyn->calcSlotineLiRegressorDq(jacs, jacDots, jacsDq, jacDotsDq, adjs, ads,
                                                  mvns[i].first.getMu().segment(dof, dof),
                                                  mvns[i].first.getMu().segment(dof * 2, dof), regressorDq);

                    dyn->calcSlotineLiRegressorDqd(jacs, jacDots, jacsDq, adjs, mvns[i].first.getMu().segment(dof, dof),
                                                   regressorDqd);
                    dyn->calcSlotineLiRegressorDqdd(jacs, regressorDqdd);
                    Yb= Y * dyn->getRegressorProjector();
                    mvns[i].first.getMu().segment(dof * 3, dof)+= Yb * pid;
                    for(size_t i0= 0; i0 < dof; i0++) {
                        Yb= regressorDq[i0] * dyn->getRegressorProjector();
                        covDot.block(dof * 3, i0, dof, 1)= Yb * pid;
                        Yb= regressorDqd[i0] * dyn->getRegressorProjector();
                        covDot.block(dof * 3, i0 + dof, dof, 1)= Yb * pid;
                        Yb= regressorDqdd[i0] * dyn->getRegressorProjector();
                        covDot.block(dof * 3, i0 + dof + dof, dof, 1)= Yb * pid;
                    }
                    
                    cov= mvns[i].first.getInvCov().inverse();
                    cov*= covDot.transpose();
                    mvns[i].first.getInvCov().noalias()= covDot * cov;
                    cov= mvns[i].first.getInvCov();
                    mvns[i].first.getDet() = cov.determinant();
                    mvns[i].first.getInvCov()= cov.inverse();
                    }
                // }

                this->npModel.learn(xC);
            }

            size_t getDof() { return dof; }

            void setSpLimit(T sps) { this->npModel.derived().setSpLimit(sps); }
            void setLifetime(size_t lifetime){
                this->npModel.derived().setLifetime(lifetime); 
            }

            void setSPMin(T sps) { this->npModel.derived().setSPMin(sps); }
            void setDetLimit(T det) { this->npModel.derived().setDetLimit(det); }
            size_t getNumberOfModels() { return this->npModel.derived().getNumberOfModels(); }
            const VectorX<T> &getTauP() { return tauP; }
            const VectorX<T> &getTauNP() { return tauNP; }

            T getMaxSingular() { return Rho.inverse().colwise().template lpNorm<1>().maxCoeff(); }

            const VectorX<T> &getPi() { return pi; }
        };
    } // namespace SP
} // namespace ARDL
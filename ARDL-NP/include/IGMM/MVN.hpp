#pragma once
#include <Eigen/Dense>
#include "ARDL/typedefs.hpp"
#include <iostream>

namespace ARDL{
    namespace NP{
        template <typename T> class MVN {
            private:
            static constexpr T M_PI2 = M_PI*2;
            size_t dims;
            size_t vs;
            T sps, det;
            VectorX<T> mu;
            MatrixX<T> invCov;

            //temps
            T distance, pdf, w, tmp1, tmp2, iDims,oDims;
            VectorX<T> muDiff, muDot, muOld, muDiff1, v, wInvCovV,invCovDot;
            MatrixX<T> wInvCov, inverseW,inverseA;

            public:

            VectorX<T>& getMu(){
                return mu;
            }
            MatrixX<T>& getInvCov(){
                return invCov;
            }
                const T& getPDF(){
                    return pdf;
                }
                const size_t& getVs(){
                    return vs;
                }
                T& getSps(){
                    return sps;
                }

                T& getDet(){
                    return det;
                }

            public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            template<typename muD, typename covD> MVN(const Eigen::MatrixBase<muD> &x, const Eigen::MatrixBase<covD>& cov){
                dims = x.rows();
                wInvCov.resize(dims,dims); 
                inverseA.resize(dims,dims); 
                inverseW.resize(dims,dims);
                invCovDot.resize(dims);
                invCov.resize(dims,dims);
                mu.resize(dims);
                muDiff.resize(dims); muDot.resize(dims); muOld.resize(dims);muDiff1.resize(dims); v.resize(dims);wInvCovV.resize(dims);
                reset(x,cov);
            }

            MVN<T>& operator=(const MVN<T>& other){
                dims = other.dims;
                vs = other.vs;
                sps = other.sps;
                invCov = other.invCov;
                det = other.det;
                mu = other.mu;
                invCovDot.resize(dims);
                muDiff.resize(dims); muDot.resize(dims); muOld.resize(dims);muDiff1.resize(dims); v.resize(dims);wInvCovV.resize(dims);
                wInvCov.resize(dims,dims); 
                inverseA.resize(dims,dims); 
                inverseW.resize(dims,dims);
                wInvCov.setZero();
                inverseA.setZero();
                inverseW.setZero();
                return *this;
            }


            template<typename muD, typename covD> void reset(const Eigen::MatrixBase<muD> &x, const Eigen::MatrixBase<covD>& cov){
                vs = 1;
                sps = 1;
                if(cov.cols()==1){
                    invCov.setIdentity();
                    invCov *= cov.asDiagonal();
                }else{
                    invCov = cov;
                }
                det = 1.0/invCov.determinant();
                mu = x;

                wInvCov.setZero();
                inverseA.setZero();
                inverseW.setZero();
            }


            template<typename muD> T &computeDistance(const Eigen::MatrixBase<muD>& x){
                return computeDistance(x,invCov);
            }
            template<typename muD, typename covD> T &computeDistance(const Eigen::MatrixBase<muD> &x, const Eigen::MatrixBase<covD> &invSigma) {
                muDiff.head(x.rows()) = (x - mu.segment(0, x.rows()));
                distance = muDiff.head(x.rows()).dot(invSigma * muDiff.head(x.rows()));
                return distance;
            }

            template<typename muD> T &computeLikelihood(const Eigen::MatrixBase<muD> &x) {
                return computeLikelihood(x,det);
            }
            template<typename muD> T &computeLikelihood(const Eigen::MatrixBase<muD> &x, const T &det) {
                pdf = std::exp(-0.5 * distance) / (std::pow(M_PI2, (x.rows() / 2.0)) * std::sqrt(det));
                pdf = std::isnan(pdf) ? 0 : std::isinf(pdf) ? std::numeric_limits<T>::max() : pdf;
                return pdf;
            }


            template<typename muD> void incrementalUpdate(const Eigen::MatrixBase<muD> &x, const T &posterior, const T &alpha) {
                vs += 1;
                sps = alpha * sps + posterior;
                w = posterior / sps;
                muDot = muDiff * w;
                muOld = mu;
                mu += muDot;
                muDot = mu - muOld;
                muDiff1 = (x - mu);

                v = (muDiff1 * std::sqrt(w));
                wInvCov = invCov * (1.0 / (1.0 - w));
                wInvCovV = wInvCov * v;
                tmp1 = 1 + wInvCovV.dot(v);
                invCov = wInvCov - ((wInvCovV * wInvCovV.transpose()) * (1.0 / tmp1));
                invCovDot = invCov * muDot;
                tmp2 = 1 - (invCovDot.dot(muDot));
                invCov += ((invCovDot * invCovDot.transpose()) * (1.0 / tmp2));
                if (!invCov.array().isFinite().all()) {
                    std::cerr << "infinity FAIL"<<std::endl;
                    exit(-2);
                }

                det *= std::pow(1 - w, mu.rows()) * tmp1;
                det *= tmp2;
            }

            template<typename muD, typename outD> T recall(const Eigen::MatrixBase<muD> &x, const Eigen::MatrixBase<outD> &output) {
                iDims = x.rows();
                oDims = output.rows();

                inverseW.block(iDims, iDims, oDims, oDims) = invCov.block(iDims, iDims, oDims, oDims).inverse();
                muDiff.head(iDims).noalias() = (mu.head(iDims)-x);
                muDiff1.head(oDims).noalias() = invCov.block(iDims, 0, oDims, iDims)*muDiff.head(iDims);
                const_cast<Eigen::MatrixBase<outD> &>(output) = inverseW.block(iDims, iDims, oDims, oDims)*muDiff1.head(oDims);
                const_cast<Eigen::MatrixBase<outD> &>(output) += mu.segment(iDims, oDims);

                wInvCov.block(iDims,0,oDims,iDims).noalias() =  inverseW.block(iDims, iDims, oDims, oDims) * invCov.block(iDims, 0, oDims, iDims);
                inverseA.block(0, 0, iDims, iDims).noalias() = -invCov.block(iDims, 0, oDims, iDims).transpose() * wInvCov.block(iDims,0,oDims,iDims);

                inverseA.block(0, 0, iDims, iDims) += invCov.block(0, 0, iDims, iDims);
    
                computeDistance(x, inverseA.block(0, 0, iDims, iDims));
                
                return computeLikelihood(x, 1.0 / inverseA.block(0, 0, iDims, iDims).determinant());
            }

            bool toRemove(const size_t &lifetimeLimit, const T &spLimit, const T &detLimit) {
                return (vs > lifetimeLimit && sps < spLimit) || (det < detLimit);
            }

             bool validDistance(const T &limit) {
                return distance < limit;
            }
        };
    }
}
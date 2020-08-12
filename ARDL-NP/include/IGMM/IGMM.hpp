#pragma once
#include "IGMM/MVN.hpp"
#include "ARDL/Util/Math.hpp"
namespace ARDL {
    using namespace Util::Math;
    namespace NP {
        template<typename T>
        class IGMM {
             private:
            aligned_vector<std::pair<MVN<T>, T>> comps;
            T chiDistance;

            MatrixX<T> initCov;
            VectorX<T> outputAccumulator;

            aligned_vector<VectorX<T>> outputs;

            size_t dim, oDim, lifeMin, modelNo;
            T spMin, spMax, beta, detLimit, sum, alpha;

            std::vector<T> pajs, priors;

            std::pair<MVN<T>,T> tmpComponent;

             public:
            const std::vector<T> &getPriors() { return priors; }
            const std::vector<T> &getActivations() { return pajs; }
            size_t getNumberOfModels() { return modelNo; }
            aligned_vector<std::pair<MVN<T>, T>> &getModels() { return comps; }

            void setLifetime(size_t lifetime) { lifeMin= lifetime; }
            void setSpLimit(T limit) { this->spMax= limit; }
            void setBeta(T beta) { this->beta= beta; }
            void setChiDistance(T dist) { chiDistance= dist; }
            void setSPMin(T spmin) { this->spMin= spmin; }
            void setDetLimit(T detLimit) { this->detLimit= detLimit; }

             public:
             EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            template<typename covD>
            IGMM(const size_t totalDim, const size_t outputDim,
                 const T &metaChi, const Eigen::MatrixBase<covD> &initialCov):tmpComponent(MVN<T>(VectorX<T>::Zero(totalDim),VectorX<T>::Zero(totalDim)),T(0)) {
                chiDistance= chi2inv(1 - metaChi, totalDim);
                dim= totalDim;
                oDim= outputDim;
                lifeMin= 2 * totalDim;
                spMin= 1 + totalDim;
                if(initialCov.cols() == 1) {
                    initCov= initialCov.array().inverse();
                } else {
                    initCov= initialCov.inverse();
                }
                spMax= 1e7;
                beta= 0.99;
                detLimit= 1e-15;
                outputAccumulator.resize(totalDim);
                modelNo= 0;
                // tmpComponent = MVN<T>(VectorX<T>::Zero(dim),VectorX<T>::Zero(dim));
            }

            IGMM(const IGMM<T>& copy):tmpComponent(MVN<T>(VectorX<T>::Zero(copy.dim),VectorX<T>::Zero(copy.dim)),T(0)) {
                chiDistance = copy.chiDistance;
                dim= copy.dim;
                oDim= copy.oDim;
                lifeMin=copy.lifeMin;
                spMin= copy.spMin;
                initCov = copy.initCov;
                spMax= copy.spMax;
                beta= copy.beta;
                detLimit= copy.detLimit;
                outputAccumulator = copy.outputAccumulator;
                modelNo= copy.modelNo;
            }

            void preallocate(size_t size) {
                Eigen::Matrix<T, Eigen::Dynamic, 1> tmp(dim);
                tmp.setZero();

                pajs.resize(size);
                priors.resize(size);
                for(size_t i= 0; i < size; i++) {
                    comps.emplace_back(MVN<T>(tmp, initCov), T(0));
                    outputs.push_back(VectorX<T>::Zero(oDim));
                }
            }

            template<typename D>
            void addComponent(const Eigen::MatrixBase<D> &x) {
                if(modelNo < comps.size()) {
                    comps[modelNo].first.reset(x, initCov);
                    comps[modelNo].second= T(0);
                } else {
                    comps.emplace_back(MVN<T>(x, initCov), T(0));
                    outputs.push_back(VectorX<T>::Zero(oDim));
                    pajs.push_back(0);
                    priors.push_back(0);
                }
                modelNo++;
            }

            template<typename D>
            void incrementModels(const Eigen::MatrixBase<D> &x) {
                for(size_t i= 0; i < modelNo; i++) {
                    comps[i].first.incrementalUpdate(
                        x, (comps[i].first.getPDF() * comps[i].second) / sum,
                        alpha);
                }
            }

            void removeComponents() {
                for(size_t i= modelNo - 1; i > 0; i--) {
                    if(comps[i].first.toRemove(lifeMin, spMin, detLimit)) {
                        if(i != modelNo - 1) {
                            tmpComponent= comps[i];
                            comps[i]= comps[modelNo - 1];
                            comps[modelNo - 1]= tmpComponent;
                        }
                        modelNo--;
                    }
                }
                size_t i =0;
                 if(comps[i].first.toRemove(lifeMin, spMin, detLimit)) {
                        if(i != modelNo - 1) {
                            tmpComponent= comps[i];
                            comps[i]= comps[modelNo - 1];
                            comps[modelNo - 1]= tmpComponent;
                        }
                        modelNo--;
                    }
            }

            template<typename D>
            void computeDistances(const Eigen::MatrixBase<D> &x) {
                for(size_t i= 0; i < modelNo; i++) {
                    comps[i].first.computeDistance(x);
                }
            }

            template<typename D>
            void computeLikelihoods(const Eigen::MatrixBase<D> &x) {
                for(size_t i= 0; i < modelNo; i++) {
                    comps[i].first.computeLikelihood(x);
                }
            }

            void computePosteriorsSum() {
                sum= 0;
                for(size_t i= 0; i < modelNo; i++) {
                    sum+= comps[i].first.getPDF() * comps[i].second;
                }
            }

            void updatePriors() {
                sum= 0;
                for(size_t i= 0; i < modelNo; i++) {
                    sum+= comps[i].first.getSps();
                }
                alpha= 1 - (1 / (spMax));
                for(size_t i= 0; i < modelNo; i++) {
                    comps[i].second= comps[i].first.getSps() / sum;
                    priors[i]= comps[i].second;
                }
            }
            template<typename D> bool distanceCheck(const Eigen::MatrixBase<D> &x) {
                for (size_t i = 0; i < modelNo; i++) {
                    if (comps[i].first.validDistance(chiDistance)) {
                        return true;
                    }
                }
                return false;
            }

            template<typename D>
            void incrementalUpdate(const Eigen::MatrixBase<D> &x) {
                computeDistances(x);
                computeLikelihoods(x);
                if(!distanceCheck(x)) {
                    addComponent(x);
                    comps[modelNo - 1].first.computeDistance(x);

                    comps[modelNo - 1].first.computeLikelihood(x);

                    updatePriors();
                }

                computePosteriorsSum();
                incrementModels(x);
                sum= 0;
                for(size_t i= 0; i < modelNo; i++) {
                    sum+= comps[i].first.getSps();
                }

                if(sum >= this->beta * spMax) {
                    for(size_t i= 0; i < modelNo; i++) {
                        comps[i].first.getSps()=
                            comps[i].first.getSps() * this->beta;
                    }
                }
                removeComponents();
                updatePriors();
            }

            template<typename muD, typename outD> T recall(const Eigen::MatrixBase<muD> &x, const Eigen::MatrixBase<outD> &output) {
                const_cast<Eigen::MatrixBase<outD> &>(output).setZero();
                outputAccumulator = output;
                sum = 0;

                for (size_t i = 0; i < modelNo; i++) {
                    if (comps[i].first.getVs() < lifeMin) {
                        pajs[i] = 0;
                        outputs[i].setZero();
                        continue;
                    }
                    outputAccumulator.setZero();
                    pajs[i] = comps[i].first.recall(x, outputAccumulator);
                    outputs[i] = outputAccumulator;
                    pajs[i] *= comps[i].second;
                    sum += pajs[i];
                }
                if (sum < 1e-100) {
                    return -1;
                }
                for (size_t i = 0; i < modelNo; i++) {
                    const_cast<Eigen::MatrixBase<outD> &>(output) += (outputs[i] * pajs[i] / sum);
                    pajs[i] /= sum;
                }
                if (output.hasNaN()) {
                    std::cout << "sum " << sum << std::endl;
                    for (size_t i = 0; i < modelNo; i++) {
                        std::cout << "PAJS[i]" << pajs[i] << std::endl;
                    }
                    exit(-5);
                }
                return sum;
            }
        };
    } // namespace NP
} // namespace ARDL
#pragma once

#include "IGMM/IGMMVC.hpp"
#include "ARDL/Non-Parametric/NPModel.hpp"

namespace ARDL {
    namespace NP {
        template<typename T>
        class IGMMModel: public NPModel<T, IGMMModel<T>>, public IGMMVC<T> {
             public:
             EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            template<typename covD>
            IGMMModel(const size_t totalDim, const size_t outputDim,
                      const T &metaChi,
                      const Eigen::MatrixBase<covD> &initialCov)
                : NPModel<T,IGMMModel<T>>(), IGMMVC<T>(totalDim, outputDim, metaChi,
                                                   initialCov) {
                                                       std::cout<<"IGMMModel TOTALDIMS:"<<totalDim<<std::endl;
                                                       std::cout<<"outputDim TOTALDIMS:"<<outputDim<<std::endl;
                                                   }

            IGMMModel(const IGMMModel<T>& copy)
                : NPModel<T,IGMMModel<T>>(copy), IGMMVC<T>(copy) {
                }
            template<typename muD, typename outD>
            void compute(const Eigen::MatrixBase<muD> &input,
                         const Eigen::MatrixBase<outD> &output) {
                IGMMVC<T>::recall(input, output);
            }
            template<typename D>
            void learn(const Eigen::MatrixBase<D> &x) {
                IGMMVC<T>::incrementalUpdate(x);
            }
        };
    } // namespace NP
} // namespace ARDL

// REGISTER_TYPE(NPMODELS, ARDL::NP::IGMMModel<double>);
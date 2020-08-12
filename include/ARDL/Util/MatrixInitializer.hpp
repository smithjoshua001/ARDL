#pragma once
#include "ARDL/typedefs.hpp"

namespace ARDL {
    namespace Util {
        template<typename T, int k>
        void init(Jacobian<T,k> &input, size_t dof) {
            if constexpr(k==Dynamic){
            input.resize(dof);
            }
            input.setZero();
        }

        template<typename T, int k>
        void init(Regressor<T,k> &input, size_t dof) {
            if constexpr(k==Dynamic){
            input.resize(dof);
            }
            input.setZero();
        }


        template<typename T, int k>
        void init(Eigen::Matrix<T, k, 1> &input, size_t dof) {
            if constexpr(k==Dynamic){
            input.resize(dof);
            }
            input.setZero();
        }

        template<typename T, int k>
        void init(Eigen::Matrix<T, k, k> &input, size_t dof) {
            if constexpr(k==Dynamic){
            input.resize(dof,dof);
            }
            input.setZero();
        }

        template<typename T>
        void init(aligned_vector<T> &input, size_t dof) {
            input.resize(dof);
            for(size_t i= 0; i < input.size(); i++) { init(input[i], dof); }
        }


        // template<typename T>
        // void initSquareMatrix(aligned_vector<T> &input, size_t dof) {
        //     input.resize(dof);
        //     for(size_t i= 0; i < input.size(); i++) { initSquareMatrix(input[i], dof); }
        // }

        // template<typename T>
        // void init(aligned_vector<T> &input, size_t dof) {
        //     input.resize(dof);
        //     for(size_t i= 0; i < input.size(); i++) { initDOFVector(input[i], dof); }
        // }


    } // namespace Util
} // namespace ARDL
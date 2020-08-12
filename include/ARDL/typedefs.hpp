#pragma once
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
#include <map>
#include <variant>
#include "ARDL/Util/Macros.hpp"

using namespace Eigen;
namespace ARDL {
    template<typename T>
    using aligned_vector= std::vector<T, Eigen::aligned_allocator<T>>;
    template<typename K, typename T>
    using aligned_map= std::map<K, T, std::less<K>, Eigen::aligned_allocator<std::pair<const K, T>>>;
    template<typename T>
    using MatrixX= Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    template<typename T, int k=Dynamic>
    using VectorX= Eigen::Matrix<T, k, 1>;
    template<typename T>
    using Gravity= Eigen::Matrix<T, 6, 1>;

    template<typename T, int k= Dynamic>
    class Jacobian: public Matrix<T, 6, k> {
         public:

        Jacobian(int dof):Matrix<T,6,k>(6,dof){}

        Jacobian(): Matrix<T, 6, k>() {}

        template<typename OtherDerived>
        Jacobian(const Eigen::MatrixBase<OtherDerived> &other): Matrix<T, 6, k>(other) {}

        template<typename OtherDerived>
        Jacobian &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
            this->Matrix<T, 6, k>::operator=(other);
            return *this;
        }
        void resize(int size) { this->Matrix<T, 6, k>::resize(6, size); }
    };

    template<typename T, int k= Dynamic>
    class Regressor: public Matrix<T, k, std::max(-1, k * 10)> {
         public:
         Regressor(int dof): Matrix<T, k, std::max(-1, k * 10)>(dof,dof*10) {}
        Regressor(): Matrix<T, k, std::max(-1, k * 10)>() {}

        template<typename OtherDerived>
        Regressor(const Eigen::MatrixBase<OtherDerived> &other): Matrix<T, k, std::max(-1, k * 10)>(other) {}

        template<typename OtherDerived>
        Regressor &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
            this->Matrix<T, k, std::max(-1, k * 10)>::operator=(other);
            return *this;
        }
        void resize(int size) { this->Matrix<T, k, std::max(-1, k * 10)>::resize(size, size*10); }
    };
        template<typename T, int k= Dynamic, int p=Dynamic>
    class BaseRegressor: public Matrix<T, k, p> {
         public:
         BaseRegressor(int dof, int params): Matrix<T, k, p>(dof,params) {}
        BaseRegressor(): Matrix<T, k, p>() {}

        template<typename OtherDerived>
        BaseRegressor(const Eigen::MatrixBase<OtherDerived> &other): Matrix<T, k, p>(other) {}

        template<typename OtherDerived>
        BaseRegressor &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
            this->Matrix<T, k, p>::operator=(other);
            return *this;
        }
        void resize(int dof,int params) { this->Matrix<T, k, p>::resize(dof, params); }
    };
    template<typename T>
    class LinkRegressor: public Matrix<T, 6, 10> {
         public:
        LinkRegressor(): Matrix<T, 6, 10>() {}
        template<typename OtherDerived>
        LinkRegressor(const Eigen::MatrixBase<OtherDerived> &other): Matrix<T, 6, 10>(other) {}

        template<typename OtherDerived>
        LinkRegressor &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
            this->Matrix<T, 6, 10>::operator=(other);
            return *this;
        }
        void resize(int)= delete;
    };

} // namespace ARDL

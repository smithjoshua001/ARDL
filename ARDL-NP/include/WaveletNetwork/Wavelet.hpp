#pragma once
#include "ARDL/typedefs.hpp"
// #include "WaveletNetwork/WaveletNetworkData.hpp"

namespace WN{
    template <typename T> struct MultiIndex{
        int j;
        ARDL::VectorX<T> k;
        ARDL::VectorX<int> centerIndex;
        friend inline bool operator==(const MultiIndex<T>& lhs, const MultiIndex<T>& rhs){ return lhs.j==rhs.j && lhs.k==rhs.k; }
    };
template <typename T> class Wavelet{
    private:
    WN::MultiIndex<T>& index;
    ARDL::VectorX<T>& t_dist;
    T& scalingFactor;
    
    public:
    Wavelet(WN::MultiIndex<T>& i, T& scalingFactor): index(i), scalingFactor(scalingFactor){
         this->scalingFactor = std::pow(2,index.j);
    }

    template <typename xD> T compute(const Eigen::MatrixBase<xD>& x){
        t_dist = x-index.k;
        T t_dist_norm = scalingFactor * t_dist.norm();
        t_dist_norm*=t_dist_norm;
        T exp = std::exp(t_dist_norm/2.0);
        exp = (t_dist.size()-t_dist_norm)*exp;
        return exp;
    }

    const WN::MultiIndex<T>& getIndex(){
        return index;
    }
};

}
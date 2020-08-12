#pragma once
#include <Eigen/Dense>
#include "ARDL/typedefs.hpp"
#include <vector>
#include "WaveletNetwork/Wavelet.hpp"
#include <map>
namespace WN {

    template<typename T>
    class WaveletNetworkData {
         public:
        // temporaries
        std::vector<ARDL::VectorX<T>> t_dists;
        std::vector<T> scalingFactors;
        std::vector<ARDL::VectorX<int>> indexRadius;

        // ARDL::VectorX<T> v;
        // ARDL::VectorX<T> s;

        // network constants
        std::pair<int, int> scalingLimits;
        ARDL::VectorX<T> step;
        ARDL::VectorX<T> mincenter;

        //active cache
        std::map<MultiIndex<T>,size_t> mi_pot;

        //thresholds
        T deactivationThresh, activationThresh;

        // member variables per wavelet
        // std::vector<MultiIndex<T>> mi_pot;
        std::vector<Wavelet<T>> w_pot;
        std::vector<ARDL::MatrixX<T>> ci;
        std::vector<ARDL::MatrixX<T>> ci_prev;

        std::vector<bool> alpha;
        std::vector<bool> activated;
        std::vector<T> discountFactor;
        T decayFactor;

        T constantDiscountFactor;

        T cmax;

        size_t dof;

        size_t gridSize;

    };

} // namespace WN
#include "WaveletNetwork/WaveletNetwork.hpp"
#include <iostream>
int main() {
    WN::WaveletNetworkData<double> wnd;

    std::pair<int, int> scalingLimits{-1, 4};
    std::pair<ARDL::VectorX<double>, ARDL::VectorX<double>> qLimits;
    ARDL::VectorX<double> qLim(3), qdLim(3), qddLim(3);
    qLim << 1, 1, 1;
    qdLim << 1, 1, 1;
    qddLim << 1, 1, 1;
    qLimits.first= -qLim;
    qLimits.second= qLim;

    WN::WaveletNetwork<double> wn(wnd, scalingLimits, qLimits, qdLim, qddLim, 10, 3);
    // ARDL::VectorX<int> ind(3);
    // ind<<4,4,4;
    ARDL::VectorX<int> ind(2);
    ind<<3,3;
    wn.generateIndicies(ind, 1, 0);
    for(size_t i = 0;i<wnd.indexRadius.size();i++){
        std::cout<<wnd.indexRadius[i].transpose()<<std::endl;
    }
}
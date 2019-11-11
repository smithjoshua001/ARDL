#pragma once
#include <random>
namespace ARDL {
    namespace Util {
        namespace Random {
            static std::random_device rd;
            static std::mt19937 e2(rd());
            static std::uniform_real_distribution<> rand_dist(0, 1);
        }
    }
}

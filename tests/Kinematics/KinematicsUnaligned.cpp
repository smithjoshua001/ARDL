#include <cstddef>
using namespace std;
size_t iters = 1000;
#define EIGEN_DONT_VECTORIZE
#define EIGEN_UNALIGNED_VECTORIZE 0
#include "Kinematics/Jacobian.hpp"

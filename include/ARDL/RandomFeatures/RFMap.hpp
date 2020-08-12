#pragma once
#include <Eigen/Dense>
// #include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <random>
//adapted from https://github.com/raffaello-camoriano/iRRLS/blob/master/modules/RFmapper/src/RFmapper.cpp
using namespace Eigen;
using namespace std;
namespace ARDL {
    template <typename T> class RFMapper {
    protected:
        Matrix<T, Dynamic, 1> inFeatures, outFeatures;
        size_t d, numRF;
        Matrix<T, Dynamic, Dynamic> W;
    // Matrix<T, Dynamic, 1> b;

    public:
        void init(size_t numRF, size_t d) {
            this->d = d;
            this->numRF = numRF;
            W.resize(numRF, d);
            // b.resize(numRF);
            default_random_engine generator;
            normal_distribution<T> dist(0, 1);
            auto normal = [&](T) {return dist(generator);};
            W = Matrix<T, Dynamic, Dynamic>::NullaryExpr(numRF, d, normal);
            // b.setRandom();
            // b *= 2 * M_PI;
        }

        template <typename IDerived, typename ODerived> void applyRF(const MatrixBase<IDerived> &input, const MatrixBase<ODerived> &output) {
            const_cast<MatrixBase<ODerived> &>(output).template block(0, 0, numRF, input.cols()) = W * input;
            // const_cast<MatrixBase<ODerived> &>(output).colwise() += b;
            const_cast<MatrixBase<ODerived> &>(output).template block(numRF, 0, numRF, input.cols()) = 1 / sqrt(d) * output.template block(0, 0, numRF, input.cols()).array().sin();

            const_cast<MatrixBase<ODerived> &>(output).template block(0, 0, numRF, input.cols()) = 1 / sqrt(d) * output.template block(0, 0, numRF, input.cols()).array().cos();
        }
    };
}

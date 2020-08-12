#pragma once
#include <Eigen/Dense>

namespace ARDL {
    namespace Util {
        namespace Filters {
            /**
             * @brief Implementation of discrete first order low pass filter
             *
             * @tparam T Type of input
             */
            template <typename T> class DiscreteLowPassFilter {
            public:
                DiscreteLowPassFilter(T cutoffFrequency, T samplingRate, size_t rows, size_t cols) : DiscreteLowPassFilter<T>(cutoffFrequency, samplingRate) {
                    buffer.resize(rows, cols);
                    initial.resize(rows, cols);
                }
                DiscreteLowPassFilter(T cutoffFrequency, T samplingRate, size_t size) : DiscreteLowPassFilter<T>(cutoffFrequency, samplingRate) {
                    buffer.resize(size, size);
                    initial.resize(size, size);
                }
                DiscreteLowPassFilter(T cutoffFrequency, T samplingRate) {
                    setCutoffAndSampling(cutoffFrequency, samplingRate);
                }
                template <typename Derived> Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> compute(const Eigen::MatrixBase<Derived> &newInput) {
                    static_assert(std::is_same<typename Derived::Scalar, T>::value, "must be same scalar");
                    buffer *= gamma;
                    buffer.noalias() += (1-gamma)*newInput;
                    //buffer = (1 - gamma) * newInput + gamma * buffer;
                    return buffer;
                }
                template <typename Derived> void setBuffer(const Eigen::MatrixBase<Derived> &buf) {
                    buffer = buf;
                    initial = buf;
                }
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getResult() {
                    return buffer;
                }
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getInitial() {
                    return initial;
                }
                void setCutoffAndSampling(T &cutoff, T &sampling) {
                    this->cutoff = cutoff;
                    //https://en.wikipedia.org/wiki/Low-pass_filter
                    //RC = 1/(2*PI*f_c)
                    T tmp1 = 1 / (cutoff * 2 * M_PI);
                    //DT = 1/sampling
                    T tmp2 = 1 / sampling;
                    //Gamma (alpha) = DT/(RC+DT)
                    gamma = tmp2 / (tmp1 + tmp2);
                    samplingTime = 1.0 / sampling;
                }
                T getBeta() {
                    //https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8460856 (23)
                    return (1 - gamma) * (1 / gamma) * (1 / samplingTime);
                }
                T getCutoff(){
                    return cutoff;
                }
                T getSamplingRate(){
                    return 1.0/samplingTime;
                }
            private:
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> buffer;
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> initial;
                T gamma;
                T samplingTime;
                T cutoff;
            };
        }
    }
}

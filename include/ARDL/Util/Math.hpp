#pragma once
#include <Eigen/Dense>

namespace ARDL {
    namespace Util {
        namespace Math {
            /**
             * @brief Turns a vector into a skew-symmetric matrix
             *
             * @tparam Derived Type of vector to allow for block/segment calls
             * @tparam Derived2 Type of matrix to allow for block/segment calls
             * @param vector Input 3x1 vector
             * @param matrix Output 3x3 matrix
             */
            template<typename Derived, typename Derived2>
            static void skewMatrix(const Eigen::MatrixBase<Derived> &vector,
                                   Eigen::MatrixBase<Derived2> const &matrix) {
                typedef typename Derived::Scalar Scalar;
                const_cast<Scalar &>(matrix(2, 1))= vector(0);
                const_cast<Scalar &>(matrix(1, 2))= -vector(0);
                const_cast<Scalar &>(matrix(0, 2))= vector(1);
                const_cast<Scalar &>(matrix(2, 0))= -vector(1);
                const_cast<Scalar &>(matrix(0, 1))= -vector(2);
                const_cast<Scalar &>(matrix(1, 0))= vector(2);
                const_cast<Eigen::MatrixBase<Derived2>&>(matrix).diagonal().setZero();
            }

            /**
             * @brief Turns a vector into the double tilde matrix (On the closed
             form computation of the dynamic matrices and their differentiation
             eq. (12))
             *
             * @tparam Derived Type of vector to allow for block/segment calls
             * @tparam Derived2 Type of matrix to allow for block/segment calls
             * @param vector Input 3x1 vector
             * @param matrix Output 3x6 matrix
             */
            template<typename Derived, typename Derived2>
            static void tildeMatrix(const Eigen::MatrixBase<Derived> &vector,
                                    Eigen::MatrixBase<Derived2> const &matrix) {
                typedef typename Derived::Scalar Scalar;
                const_cast<Eigen::MatrixBase<Derived2> &>(matrix).setZero();
                const_cast<Scalar &>(matrix(0, 0))= vector(0);
                const_cast<Scalar &>(matrix(1, 1))= vector(1);
                const_cast<Scalar &>(matrix(2, 2))= vector(2);
                const_cast<Scalar &>(matrix(1, 3))= vector(0);
                const_cast<Scalar &>(matrix(2, 4))= vector(0);
                const_cast<Scalar &>(matrix(0, 3))= vector(1);
                const_cast<Scalar &>(matrix(2, 5))= vector(1);
                const_cast<Scalar &>(matrix(0, 4))= vector(2);
                const_cast<Scalar &>(matrix(1, 5))= vector(2);
            }

            /**
             * @brief Analytic sign function
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Steepness of output change around 0
             * @return int The sign of the input value
             */
            template<typename T>
            static int sgn(const T &val, const T &eps= T(1e-50)) {
                return val / std::sqrt(std::pow(val, 2) + eps);
            }

            /**
             * @brief Analytic derivative of sign function
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Steepness of output change around 0
             * @return int The derivative of the sign of the input value
             */
            template<typename T>
            static T sgnDeriv(const T &val, const T &eps= T(1e-50)) {
                return eps / std::pow(std::pow(val, 2) + eps, 1.5);
            }

            /**
             * @brief Branchless sign function using bool comparisons
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Deadzone around 0 for the sign function
             * @return int The sign of the input value
             */
            template<typename T>
            static int sgnIF(const T &val, const T &eps= T(0)) {
                return (T(eps) < val) - (val < T(-eps));
            }

            /**
             * @brief Branchless deadzone function using bool comparisons and
             * inversions. Deadzone is a function that calculates a deadzone
             * around zero for a number. If the number lies within the bounds
             * this function returns 0 otherwise it returns 1.
             *
             * @tparam T Type of input
             * @param val Input number
             * @param eps Deadzone around 0
             * @return int 0 if input is between the \pm eps bounds otherwise 1
             */
            template<typename T>
            static int deadzone(const T &val, const T &eps= T(0)) {
                return !((T(eps) > val) && (val > T(-eps)));
            }

            static constexpr std::array<double, 14> COF= {
                57.1562356658629235,      -59.5979603554754912,
                14.1360979747417471,      -0.491913816097620199,
                0.339946499848118887e-4,  0.465236289270485756e-4,
                -0.983744753048795646e-4, 0.158088703224912494e-3,
                -0.210264441724104883e-3, 0.217439618115212643e-3,
                -0.164318106536763890e-3, 0.844182239838527433e-4,
                -0.261908384015714087e-4, 0.368991826595316234e-5};

            static constexpr double p0= -0.322232431088;
            static constexpr double p1= -1.0;
            static constexpr double p2= -0.342242088547;
            static constexpr double p3= -0.204231210245e-1;
            static constexpr double p4= -0.453642210148e-4;
            static constexpr double q0= 0.993484626060e-1;
            static constexpr double q1= 0.588581570495;
            static constexpr double q2= 0.531103462366;
            static constexpr double q3= 0.103537752850;
            static constexpr double q4= 0.38560700634e-2;
            static constexpr double OFLO= 10e30;
            static constexpr double E= 10e-6;

            static double gammaln(double xx) {
                double y= xx;
                double x= xx;
                double tmp= x + 5.2421875;
                tmp= (x + 0.5) * std::log(tmp) - tmp;
                double ser= 0.999999999999997092;
                for(int i= 0; i < 14; i++) { ser+= COF[i] / ++y; }
                return tmp + std::log(2.5066282746310005 * ser / x);
            }

            static double gauinv(double p) {
                if(p == 0.5) { return 0.0; }
                double ps= p;
                if(ps > 0.5) { ps= 1 - ps; }
                double yi= std::sqrt(std::log(1.0 / (ps * ps)));
                double gauinv=
                    yi + ((((yi * p4 + p3) * yi + p2) * yi + p1) * yi + p0) /
                             ((((yi * q4 + q3) * yi + q2) * yi + q1) * yi + q0);
                if(p < 0.5) {
                    return -gauinv;
                } else {
                    return gauinv;
                }
            }
            static double gammaintegral(double p, double x) {
                double g= gammaln(p);
                double factor= std::exp(p * std::log(x) - x - g);
                double gin;
                if((x > 1.0) && (x > p)) {
                    bool end= false;
                    double a= 1.0 - p;
                    double b= a + x + 1.0;
                    double term= 0.0;
                    double pn[6];
                    pn[0]= 1.0;
                    pn[1]= x;
                    pn[2]= x + 1.0;
                    pn[3]= x * b;
                    gin= pn[2] / pn[3];
                    do {
                        double rn;
                        a++;
                        b= b + 2.0;
                        term++;
                        double an= a * term;
                        for(int i= 0; i <= 1; i++) {
                            pn[i + 4]= b * pn[i + 2] - an * pn[i];
                        }
                        if(pn[5] != 0.0) {
                            rn= pn[4] / pn[5];
                            double diff= std::abs(gin - rn);
                            if(diff < E * rn) {
                                end= true;
                            } else {
                                gin= rn;
                            }
                        }
                        if(!end) {
                            for(int i= 0; i < 4; i++) { pn[i]= pn[i + 2]; }
                            if(std::abs(pn[5]) >= OFLO) {
                                for(int i= 0; i < 4; i++) {
                                    pn[i]= pn[i] / OFLO;
                                }
                            }
                        }
                    } while(!end);
                    gin= 1.0 - factor * gin;
                } else {
                    gin= 1.0;
                    double term= 1.0;
                    double rn= p;
                    do {
                        rn++;
                        term= term * x / rn;
                        gin= gin + term;
                    } while(term > E);
                    gin= gin * factor / p;
                }
                return gin;
            }
            /** TAKEN FROM
             * https://github.com/jchambyd/FIGMN/blob/7b4daf375c01688a4fed5592bc1a383adb05e08b/FIGMN/src/liac/igmn/util/ChiSquareUtils.java
             * Returns the inverse chi-squared distribution. Uses the method
             * given in Best and Roberts 1975. Makes calls to private functions
             * using the methods of Bhattacharjee 1970 and Odeh and Evans 1974.
             * All converted to Java by the author (yes, the author knows
             * FORTRAN!)
             *
             * @param p The p-value
             * @param v The number of degrees of freedom
             * @return The percentage point
             */
            static double chi2inv(double p, double v) {
                if(p < 0.000002) { return 0.0; }
                if(p > 0.999998) { p= 0.999998; }

                double xx= 0.5 * v;
                double c= xx - 1.0;
                double aa= std::log(2);
                double g= gammaln(v / 2.0);
                double ch;
                if(v > (-1.24 * std::log(p))) {
                    if(v > 0.32) {
                        // 3
                        double x= gauinv(p);
                        double p1= 0.222222 / v;
                        ch= v * std::pow(x * std::sqrt(p1) + 1.0 - p1, 3);
                        if(ch > (2.2 * v + 6.0)) {
                            ch= -2.0 * (std::log(1.0 - p) -
                                        c * std::log(0.5 * ch) + g);
                        }
                    } else {
                        // 1+2
                        ch= 0.4;
                        double q;
                        double a= std::log(1.0 - p);
                        do {
                            q= ch;
                            double p1= 1.0 + ch * (4.67 + ch);
                            double p2= ch * (6.73 + ch * (6.66 + ch));
                            double t= -0.5 + (4.67 + 2.0 * ch) / p1 -
                                      (6.73 + ch * (13.32 + 3.0 * ch)) / p2;
                            ch= ch -
                                (1.0 - std::exp(a + g + 0.5 * ch + c * aa) *
                                           p2 / p1) /
                                    t;
                        } while(std::abs(q / ch - 1.0) >= 0.01);
                    }
                } else {
                    // START
                    ch= std::pow(p * xx * std::exp(g + xx * aa), 1.0 / xx);
                }
                double q;
                do {
                    // 4 + 5
                    q= ch;
                    double p1= 0.5 * ch;
                    double p2= p - gammaintegral(xx, p1);
                    double t=
                        p2 * std::exp(xx * aa + g + p1 - c * std::log(ch));
                    double b= t / ch;
                    double a= 0.5 * t - b * c;
                    double s1=
                        (210.0 +
                         a * (140.0 +
                              a * (105.0 +
                                   a * (84.0 + a * (70.0 + 60.0 * a))))) /
                        420.0;
                    double s2= (420.0 +
                                a * (735.0 +
                                     a * (966.0 + a * (1141.0 + 1278.0 * a)))) /
                               2520.0;
                    double s3= (210.0 + a * (462.0 + a * (707.0 + 932.0 * a))) /
                               2520.0;
                    double s4= (252.0 + a * (672.0 + 1182.0 * a) +
                                c * (294.0 + a * (889.0 + 1740.0 * a))) /
                               5040.0;
                    double s5=
                        (84.0 + 264.0 * a + c * (175.0 + 606.0 * a)) / 2520.0;
                    double s6= (120.0 + c * (346.0 + 127.0 * c)) / 5040.0;
                    ch= ch +
                        t * (1.0 + 0.5 * t * s1 -
                             b * c *
                                 (s1 -
                                  b * (s2 -
                                       b * (s3 -
                                            b * (s4 - b * (s5 - b * s6))))));
                } while(std::abs(q / ch - 1.0) > E);
                return ch;
            }
        } // namespace Math
    }     // namespace Util
} // namespace ARDL

#pragma once
#include <Eigen/Dense>
#include "ARDL/typedefs.hpp"

namespace ARDL {
    template<typename T>
    class Pseudoinverse {
         private:
        MatrixX<T> pinv;
        Eigen::ColPivHouseholderQR<MatrixX<T>> qr;
        MatrixX<T> C, R;
        size_t r;

         public:
        Pseudoinverse(size_t r, size_t c) {
            qr= Eigen::ColPivHouseholderQR<MatrixX<T>>(MatrixX<T>::Identity(r, c));
            C= MatrixX<T>::Zero(std::min(r, c), std::min(r, c));
            pinv= MatrixX<T>::Zero(c, r);
            R= MatrixX<T>::Zero(std::max(r, c), std::max(r, c));
        }
        template<typename D>
        MatrixX<T> &compute(const Eigen::MatrixBase<D> &input, T thresh= T(1e-10)) {
            qr.setThreshold(thresh);
            qr.compute(input);
            r= 0;
            r= qr.rank();
            //     std::cout<<"RANK: "<<r<<std::endl;
            //    std::cout<<"QRINPUT\n"<<input<<std::endl<<std::endl;
            //     std::cout<<qr.householderQ()*qr.matrixR()*qr.colsPermutation().transpose()<<std::endl<<std::endl;
            R.setZero();
            R.block(0, 0, r, r)= qr.matrixR().block(0, 0, r, r).template triangularView<Eigen::Upper>();
            ginv(R.block(0, 0, r, r));
            // std::cout << "QRTEST\n"
            //           << R.block(0, 0, r,r) * pinv.block(0, 0, r, r)<< std::endl<<qr.matrixR().cols() << std::endl;
            R.setZero();
            //
            R.block(0, 0, r, r)= pinv.block(0, 0, r, r);
            // R.block(0, r, qr.matrixR().cols(), qr.householderQ().rows() - r).setZero();
            // std::cout<<"householderQ rows "<<qr.householderQ().rows()<<std::endl;
            // std::cout<<R.block(0,0,qr.matrixR().cols(),qr.householderQ().rows())<<std::endl<<std::endl;
            //
            // std::cout << R.block(0, 0, qr.matrixR().cols(), qr.householderQ().rows()) * qr.householderQ().transpose()
            //           << std::endl
            //           << std::endl;
            pinv= qr.colsPermutation() * R.block(0, 0, qr.matrixR().cols(), qr.householderQ().rows()) *
                  qr.householderQ().transpose();
            // .block(0,0,qr.householderQ().rows(),r)
            // std::cout<<"FINAL\n"<<input*pinv<<std::endl<<std::endl<<std::endl;
            // std::cout<<"EQ\n"<<input.transpose()*(input*input.transpose()).inverse()<<std::endl<<std::endl;
            // pinv = input.transpose()*(input*input.transpose()).inverse();
            return pinv;
        }

        template<typename D>
        void ginv(const Eigen::MatrixBase<D> &input) {
            if(input.rows() > input.cols()) {
                C.block(0, 0, input.cols(), input.cols()).noalias()= input.transpose() * input;
                pinv.block(0, 0, input.cols(), input.rows()).noalias()=
                    C.block(0, 0, input.cols(), input.cols()).inverse() * input.transpose();
            } else {
                C.block(0, 0, input.rows(), input.rows()).noalias()= input * input.transpose();
                pinv.block(0, 0, input.cols(), input.rows()).noalias()=
                    C.block(0, 0, input.rows(), input.rows()).colPivHouseholderQr().solve(input).transpose();

                // (C.block(0, 0, input.rows(), input.rows()).inverse() * input).transpose();
            }
        }
    };
    template<typename T>
    class PseudoinverseSVD {
         private:
        MatrixX<T> pinv;
        Eigen::JacobiSVD<MatrixX<T>> svd;
        VectorX<T> R;

         public:
        PseudoinverseSVD(size_t r, size_t c) {
            svd= Eigen::JacobiSVD<MatrixX<T>>(MatrixX<T>::Identity(r, c), Eigen::ComputeThinU | Eigen::ComputeThinV);
            pinv= MatrixX<T>::Zero(c, r);
            R= MatrixX<T>::Zero(std::max(r, c), std::max(r, c));
        }
        template<typename D>
        MatrixX<T> &compute(const Eigen::MatrixBase<D> &input, T thresh= T(1e-10)) {
           svd.setThreshold(thresh);
           svd.compute(input);
           R = svd.singularValues();
           for(size_t i = 0; i<svd.rank();i++){
                   R(i) = 1.0/R(i);
           }
           pinv = svd.matrixV().block(0,0,pinv.rows(),svd.rank())*R.head(svd.rank()).asDiagonal()*svd.matrixU().block(0,0,pinv.cols(),svd.rank()).transpose();
                  
                  return pinv;
        }

    };
    namespace Util {
        template<typename T>
        class PseudoInverseQR {
             private:
            static Eigen::ColPivHouseholderQR<ARDL::MatrixX<T>> qr;
            static ARDL::MatrixX<T> R, Q, RTRRT, Inv;

             public:
            template<typename D>
            static ARDL::MatrixX<T> computePinv(const Eigen::MatrixBase<D> &input) {
                qr= input.colPivHouseholderQr();
                qr.setThreshold(1e-6);
                qr.compute(input);
                R= qr.matrixR().block(0, 0, qr.rank(), qr.matrixR().cols());
                Q= qr.matrixQ().block(0, 0, qr.matrixQ().rows(), qr.rank());
                RTRRT= R.transpose().ldlt().solve(R * R.transpose());
                Inv= qr.colsPermutation() * RTRRT * Q.transpose();
                return Inv;
            }
            static ARDL::MatrixX<T> computePinv2(ARDL::MatrixX<T> &input) {
                Eigen::ColPivHouseholderQR<ARDL::MatrixX<T>> qr(input);
                qr.setThreshold(1e-6);
                qr.compute(input);
                ARDL::MatrixX<T> R= qr.matrixR().template block(0, 0, qr.rank(), qr.matrixR().cols());
                ARDL::MatrixX<T> Q= qr.matrixQ();
                ARDL::MatrixX<T> RTRRT= (R * R.transpose()).ldlt().solve(R.transpose());
                ARDL::MatrixX<T> Inv=
                    qr.colsPermutation() * RTRRT * Q.template block(0, 0, qr.matrixQ().rows(), qr.rank()).transpose();
                return Inv;
            }
        };
    } // namespace Util
} // namespace ARDL
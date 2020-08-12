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
            pinv = MatrixX<T>::Zero(c,r);
            R= MatrixX<T>::Zero(std::max(r, c), std::max(r, c));
        }
        template <typename D> MatrixX<T> &compute(const Eigen::MatrixBase<D> &input, T thresh= T(1e-10)) {
            qr.setThreshold(thresh);
            qr.compute(input);
            r=0;
            r = qr.rank();
        //     std::cout<<"RANK: "<<r<<std::endl;
        //    std::cout<<"QRINPUT\n"<<input<<std::endl<<std::endl;
        //     std::cout<<qr.householderQ()*qr.matrixR()*qr.colsPermutation().transpose()<<std::endl<<std::endl;
        R.setZero();
            R.block(0,0,r,qr.matrixR().cols()) = qr.matrixR().block(0,0,r,qr.matrixR().cols()).template triangularView<Eigen::Upper>();
            ginv(R.block(0,0,r,qr.matrixR().cols()));
        R.setZero();
            // std::cout<<"QRTEST\n"<<R.block(0,0,r,qr.matrixR().cols())*pinv.block(0,0,qr.matrixR().cols(),r)<<std::endl;
            R.block(0,0,qr.matrixR().cols(),r) = pinv.block(0,0,qr.matrixR().cols(),r);
            R.block(0,r,qr.matrixR().cols(),qr.householderQ().rows()-r).setZero();
            // std::cout<<"householderQ rows "<<qr.householderQ().rows()<<std::endl;
            // std::cout<<R.block(0,0,qr.matrixR().cols(),qr.householderQ().rows())<<std::endl<<std::endl;
            // std::cout<<R.block(0,0,qr.matrixR().cols(),qr.householderQ().rows())*qr.householderQ().transpose()<<std::endl<<std::endl;
            pinv = qr.colsPermutation()*R.block(0,0,qr.matrixR().cols(),qr.householderQ().rows())*qr.householderQ().transpose();
            // .block(0,0,qr.householderQ().rows(),r)
            // std::cout<<"FINAL\n"<<pinv<<std::endl<<std::endl<<std::endl;
            // std::cout<<"EQ\n"<<input.transpose()*(input*input.transpose()).inverse()<<std::endl<<std::endl;
            // pinv = input.transpose()*(input*input.transpose()).inverse();
            return pinv;
        }

        template <typename D> void ginv(const Eigen::MatrixBase<D> &input) {
            if(input.rows() > input.cols()) {
                C.block(0,0,input.cols(),input.cols()).noalias()= input.transpose() * input;
                pinv.block(0,0,input.cols(),input.rows()).noalias()= C.block(0,0,input.cols(),input.cols()).inverse() * input.transpose();
            } else {
                C.block(0,0,input.rows(),input.rows()).noalias()= input * input.transpose();
                pinv.block(0,0,input.cols(),input.rows()).noalias()= (C.block(0,0,input.rows(),input.rows()).inverse() * input).transpose();
            }
        }
    };
} // namespace ARDL
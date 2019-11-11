#pragma once
#include "ARDL/Util/Math.hpp"

namespace ARDL {
    using namespace Util::Math;
    namespace Regressors {
        template<typename Derived, typename Derived2> static void momentumRegressor(const Eigen::MatrixBase<Derived> &vector, Eigen::MatrixBase<Derived2> const &regressorMat) {
            const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).setZero();
            const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 1>(0, 0) = vector.template head<3>();
            skewMatrix(vector.template tail<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 3>(0, 1));
            skewMatrix(-vector.template head<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 3>(3, 1));
            tildeMatrix(vector.template tail<3>(), const_cast<Eigen::MatrixBase<Derived2> &>(regressorMat).template block<3, 6>(3, 4));
        }

        template<typename Derived, typename Derived2> static void frictionRegressor(const Eigen::MatrixBase<Derived> &qd, Eigen::MatrixBase<Derived2> const &out) {
            const_cast< Eigen::MatrixBase<Derived2> & >(out).setZero();
            for (int i = 0; i < qd.rows(); i++) {
                const_cast< Eigen::MatrixBase<Derived2> & >(out)(i, i) = qd;
                const_cast< Eigen::MatrixBase<Derived2> & >(out)(i, i + qd.rows()) = sgn(qd, 1e-30f);
            }
        }
    }
}

#pragma once
#include <ostream>

#include "ARDL/Util/Math.hpp"

// namespace ARDL {
//     using namespace Util::Math;
//     namespace Math {
//         template <typename Exp1, typename Exp2> class LieProduct : public LieExpression<LieProduct<Exp1, Exp2> > {
//         Exp1 const &left;
//         Exp2 const &right;
//         public:
//             template <typename Exp> inline void operator()(LieExpression<Exp> &out) const {
//                 std::cout << "Operator LieExpression" << std::endl;
//                 if constexpr (std::is_same<Exp1, Pose>) {
//                     right(out);
//                     left.apply(out, out);
//                 } else if constexpr (std::is_same<Exp2, Pose>) {
//                     left(out);
//                     right.applyTo(out, out);
//                 } else {
//                     std::cout << "Double expansion needed" << std::endl;
//                 }
//             }
//             template <typename Derived> inline void operator()(const Eigen::MatrixBase<Derived> &out) const {
//                 std::cout << "Operator MatrixBase" << std::endl;
//             }
//         }
//     }
// }

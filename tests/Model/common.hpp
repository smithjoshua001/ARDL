#pragma once
#include <Eigen/Dense>
template <typename Derived, typename Dervied2> void inline checkApproxMatrix(const Eigen::MatrixBase<Derived> &first, const Eigen::MatrixBase<Dervied2> &second, double margin = 0.0, double eps = std::numeric_limits<float>::epsilon() *100) {
    for (int i = 0; i < first.size() - 1; i++) {
        CHECK(first(i) == Approx(second(i)).margin(margin).epsilon(eps));
    }
    REQUIRE(first(first.size() - 1) == Approx(second(first.size() - 1)).margin(margin).epsilon(eps));
}

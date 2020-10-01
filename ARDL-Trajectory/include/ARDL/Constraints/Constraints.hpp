#pragma once

#include <variant>

#include "ARDL/Constraints/Constraint.hpp"
#include "ARDL/Constraints/JointPositionLimit.hpp"
#include "ARDL/Constraints/JointVelocityLimit.hpp"
#include "ARDL/Constraints/SelfCollision.hpp"
#include "ARDL/Constraints/CartesianLink.hpp"
namespace ARDL {
    namespace Constraints {
        template <typename T> using ConstraintsVariant = std::variant <SelfCollision<T>, JointPositionLimit<T>, JointVelocityLimit<T>, CartesianLink<T> >;
    }
}

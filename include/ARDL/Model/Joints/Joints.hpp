#pragma once
#include "ARDL/Model/Joints/FixedJoint.hpp"
#include "ARDL/Model/Joints/RevoluteJoint.hpp"
#include <variant>

namespace ARDL {
    namespace Model {
        namespace Joints {
            #if ARDL_VARIANT == ON
                template <typename T> using JointVariant = std::variant <FixedJoint<T>, RevoluteJoint<T> >;
                template <typename T> using JointVariantPtr = std::variant <FixedJoint<T> *, RevoluteJoint<T> * >;

                template <typename T> using JointVariantSharedPtr = std::variant <std::shared_ptr<FixedJoint<T> >, std::shared_ptr<RevoluteJoint<T> > >;
            #else
                template <typename T> using JointVariant = Joint<T>;
                template <typename T> using JointVariantPtr = Joint<T> *;

                template <typename T> using JointVariantSharedPtr = std::shared_ptr<Joint<T> >;
            #endif
        }
    }
}

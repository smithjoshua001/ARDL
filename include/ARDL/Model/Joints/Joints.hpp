#pragma once
#include "ARDL/Model/Joints/FixedJoint.hpp"
#include "ARDL/Model/Joints/PrismaticJoint.hpp"
#include "ARDL/Model/Joints/RevoluteJoint.hpp"
#include <variant>

namespace ARDL::Model::Joints {
#if ARDL_VARIANT
template <typename T>
using JointVariant =
    std::variant<FixedJoint<T>, RevoluteJoint<T>, PrismaticJoint<T>>;
template <typename T>
using JointVariantPtr =
    std::variant<FixedJoint<T> *, RevoluteJoint<T> *, PrismaticJoint<T> *>;

template <typename T>
using JointVariantSharedPtr = std::variant<std::shared_ptr<FixedJoint<T>>,
                                           std::shared_ptr<RevoluteJoint<T>>,
                                           std::shared_ptr<PrismaticJoint<T>>>;
#else
template <typename T> using JointVariant = Joint<T>;
template <typename T> using JointVariantPtr = Joint<T> *;

template <typename T> using JointVariantSharedPtr = std::shared_ptr<Joint<T>>;
#endif
} // namespace ARDL::Model::Joints

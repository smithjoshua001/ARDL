#pragma once
#include "ARDL/Util/Visit.hpp"

#if ARDL_VARIANT == ON
    #define ARDL_VISIT my_visit
    #define VISITORDER 0

    #if VISITORDER
        #define ARDL_visit(member, function)                                                                           \
            ARDL_VISIT(                                                                                                \
                member, [&](auto &x) -> auto { return x.function; })

        #define ARDL_visit_ptr(member, function)                                                                       \
            ARDL_VISIT(                                                                                                \
                member, [&](auto *x) -> auto { return x->function; })
    #else
        #define ARDL_visit(member, function)                                                                           \
            ARDL_VISIT(                                                                                                \
                [&](auto &x) -> auto { return x.function; }, member)
        #define ARDL_visit_ptr(member, function)                                                                       \
            ARDL_VISIT(                                                                                                \
                [&](auto *x) -> auto { return x->function; }, member)
    #endif

    #define ARDL_NTYPE_CLASS_INHERIT(NTYPE, NAME, INHERIT)                                                             \
        template<typename NTYPE>                                                                                       \
        class NAME: public INHERIT<NTYPE, NAME<NTYPE>>
    #define ARDL_NTYPE_CLASS_BASE(NTYPE, NAME)                                                                         \
        template<typename NTYPE, typename Derived>                                                                     \
        class NAME

    #define ARDL_DERIVED(NAME)                                                                                         \
        constexpr Derived &derived() { return static_cast<Derived &>(*this); }

    #define ARDL_BASE_TYPE(NTYPE, NAME) NAME<NTYPE, Derived>
    #define ARDL_INHERIT_TYPE(NTYPE, NAME, INHERIT) INHERIT<NTYPE, NAME<NTYPE>>
#else
    #define ARDL_visit(member, function) (member).function
    #define ARDL_visit_ptr(member, function) member->function
    #define ARDL_NTYPE_CLASS_INHERIT(NTYPE, NAME, INHERIT)                                                             \
        template<typename NTYPE>                                                                                       \
        class NAME: public INHERIT<NTYPE>

    #define ARDL_NTYPE_CLASS_BASE(NTYPE, NAME)                                                                         \
        template<typename NTYPE>                                                                                       \
        class NAME

    #define ARDL_DERIVED(NAME)                                                                                         \
        NAME &derived() { return *this; }

    #define ARDL_BASE_TYPE(NTYPE, NAME) NAME<NTYPE>
    #define ARDL_INHERIT_TYPE(NTYPE, NAME, INHERIT) INHERIT<NTYPE>
#endif

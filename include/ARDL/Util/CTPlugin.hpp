#pragma once
#include <variant>
// : Rank<N - 1>
#define CREATE_PLUGIN_TYPE(pName, pType, maxNum) \
namespace ns_##pName {\
enum { kMaxRegisteredTypes = maxNum };\
\
template <int N>\
struct Rank : Rank<N-1> {};\
\
template <>\
struct Rank<0> {};\
\
template <class... Ts>\
struct TypeList {\
  static const int size = sizeof...(Ts);\
  using variant = std::variant<Ts...>;\
};\
\
template <class List, class T>\
struct Append;\
\
template <class... Ts, class T>\
struct Append<TypeList<Ts...>, T> {\
  typedef TypeList<Ts..., pType<T>> type;\
  static_assert(std::is_convertible<T*,pType<T>*>::value, "Type is not of base type");\
};\
template <class Tag>\
TypeList<> GetTypes(Tag*, Rank<0>) { return {}; }\
}\
struct pName

// Evaluates to TypeList of all types previously registered with
// REGISTER_TYPE macro with the same tag.
#define GET_REGISTERED_TYPES(Tag) \
  decltype(ns_##Tag::GetTypes(static_cast<Tag*>(nullptr), ns_##Tag::Rank<ns_##Tag::kMaxRegisteredTypes>()))

// Appends Type to GET_REGISTERED_TYPES(Tag).
#define REGISTER_TYPE(Tag, Type)                              \
  namespace ns_##Tag{\
  inline ns_##Tag::Append<GET_REGISTERED_TYPES(Tag), Type>::type        \
  GetTypes(Tag*, ns_##Tag::Rank<GET_REGISTERED_TYPES(Tag)::size + 1>) { \
    return {};                                                \
  }                                                           \
  }\
  static_assert(GET_REGISTERED_TYPES(Tag)::size < ns_##Tag::kMaxRegisteredTypes,"Reached max number of plugins.");     \
  static_assert(true, "")
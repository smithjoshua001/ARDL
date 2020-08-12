#pragma once
/**********************************************/
/* Alternative std::visit implementation      */
/* my_visit                                   */
/*                                            */
/* handling of return void missing            */
/*https://godbolt.org/#z:OYLghAFBqd5TKALEBjA9gEwKYFFMCWALugE4A0BIEAViAIzkA2AhgHaioCkATAEK8%2B5AM7oArqVTYQAcgD0AKiXKVqteo2aFcrgAYAgooDU%2BpkWyk2LIgQBu2I8KKYQIWwWHEjBALYAHJmwfbDYiawJ0NiNomO09QwUjHwBPAH13TyIY7JzcvPzouINjAtKy8qMihKMkdkwmAg4jdAAzI1JsIgko23QCTCSPTyayqsUtCcnJnQN43gBmRtQmMRwjLnmAYVQnTBwWjdw5nkW2ZdWHDc2WsTObSJYmQ%2BPT87WromS/bFSiUhZiMJnrMTksVu8trYWKQCOwiMD9PE5HIjHwWJ5UEYMGwnNgAB5%2BUhGaH/ZLefyBYKhcKReLmCnWS5bT7fKzBIwAFXIjmcrk8AC8flk2AjlujhFjIriCaRUiSWMk5gB2AQGPxiABGDVQIHi2Wx0sJkpx5hlctIpIgAEo9TEuCrbdF7QAROYGfVS01Gg1e2Xy5IQH34wnm0nGpzrHgANmaRCQFht7rtDqTuRaZCMEBYYhI3nW82dPJcIAFQvtfF0Lo2fDzYDAG0LIvmNcEggIif0%2BXLjoKLC4AFY%2BAQB4WG7H46QAHR9wfD/uu5s97JV1PrJWu2aroMy8O%2B0MKyMx9Dff4kUgNwOe4N%2Bi0Hn2H8cJnvd1fZdNErM59B5se7PkEQVfnLSt12rWt6wLIwmxbfg2w7LsU07MoZyHEd80LdA4wsacB1Q%2BdqyXZMNyQ3IOi6SxKjjDwCNXFdES3K8d2zXNjwsawyFwkcID/EsAKFbwrV3QjogAWkOTld0PZ9EPyMjumJXC5wXVUSLXYie23I1mO/VjTw4wcuJ40tfgE4SjDE%2BZcAk3go2klTZM6eSUKUmjVLowk7EZXVVw5BTBxFed0LXARQMXWZQpUpEUU0ol4yYE8jFzP52GEd8fCSMQzAIAIHEaQgpAlXMWEcRpgECKLvDYHA8WJKqjA1FhUAAa2JYAARFAx6QCRl802Iy%2BN%2BSchqMQgfCBSz4icUgxFQLIfEymx9zSPL8WVeyYmRIwWkas9iT8AICGwAYkvjSrquaNoqIlUaQk8WlVyccJMRi4kvzO/EADEdrIbjeV4wCslGgBJKr8Xg5N1tybT2k6IL%2BoB8t6CrMLVOyaHgfMIlfz%2B4z4RVEDlLMqAMYsVt%2BF6kbfBB86bJszMyMqMdRuEK0rXIIbJxtFGHPIqIyLA6T1NXTaMB8DVGgceaGhys6CAKxLv2K4Yyty0G8R7LrWHMXqWRCFhgg5oxqbl7BxqOB6whsZ7GK0t7RfFthsAgY2CsNvKTZZqD0HxKQ/HhWiZLyaGGex4tceA5HIZydGqcxuGcYGvGK0jomIBD2DIJWvEvtmn6SfPWD%2BCtBR3YKtmOa5qPsjkij%2Be5oi3VUzbhAOrJlcCd6asaXNhGwE9GQGLOe02q7vAldhKvsUhe6225ZoiKJWklMWJZ7R6rd3a9Xp71vfrDxPl4do7BLYb28V9/23MD3IYqWq54aFblS1aDmIGZrmrNL03XPyGOfCBuOod/wI3xinV8MR3yZj/gA0mBYH5AVARFSm/8451l/HxF%2BQ036%2BBZgLQufB87g1KF/IEg585oTHPbCWmBeD9izjnM82CUGk34PQK0ch6HfVIEwmBpBK5mRrlEEhP8G6bibiiEgYQmBQTEGLCwF1iRMGkcIi2T1N5MTemwWRGoLCpFaKkEIfxDrCGtF7H2fdL7LmvjkQRncGE/Wfi0V%2B79%2BEB3UhFRuCwCAtH2EYAAqgAOU2AACVwJsAA0rgZ0qQADiuAOQvAsKQDMgSQlhMidEuJvlHgdBYJgMk%2BxqGJKYL3F43jfGpFiQEvxmxKkvEKY7fxQTQkRKibE%2BJRZXCVJwGEAgTAunpGhLCUIAzgCdGKaUkE8wGkOFSS0jJ7TfI8TGZfBYIRCAHCmes7xjdNq%2BBylSGki82gpHSB4YgdIgjdW1lcDWXw9bsg5KbIgmwCDkDuayfWDggbCAAGrUQmgYKaM05ppAyMQVI%2Byngo0uQyG5WwPkPIcE8pwrz3kW33gDQ2GQETAtmkkMF5yiCQopFcFFLy3mdJAFnVIvcACOYgQhSCuBkDmzxAWIkQprHqtzVy6zZA4f5mQyDckRQKw2vyhlwjNpNS2ctKoNEaTgZYutPwkEEqgR4TAIBrwTgDLEbyexCuIBmWm0YjDgrPOi1SkqYTSqktGbFUrQi4PClHMQysjAAFl9BAyClLRa/pIVq3vn9KEdrQi0oGrYUNxZlUKiAlsW1wyiAyssmyiaMKGImi3tDcwqKCBBXJa81wUIVjYFcj2bx6ijQQHzRSimPqgauC0XI2U%2BjDEwlNtaIhwUzLVrrc815Rg0GQVQO2My1dHIUVOZaklAReWowKDxbu2BgD/CYKkH0cJY3AMfolIdhbBBGCRpZa1FQqU0vpYys4FbIQeAzUcSyrhNVKJ1eA0o47z0Xqpe%2BAA7tCGhSaiUcUshAS1ZA2aTryDxf9gGrjJulYccDzrU1WgriIp0oFVy2JXWwXoTUnZmVg2QADpAgObGNWeZDEG%2BHftyHM9JbSslXCbXyXe9bXk2kHDi/CYGSOkDIxRxDLqaOoZZuhoarjOXuMJmIhYtxfGMdaZk%2BJuyUSPExlYGw9hyQHL1vcKIPcw1Ethdcpkmx%2BVfKMFRkViV7niuGiJ1NCJGiKocMqsw9y1XoEErOolEBbNY2jHTWj3JnMSjNVGJ1Ea0NrSrW0KA4aU3CEnGWxlgRhDCFSBqNI5i/YL1MfaTYxWK69uyHGZJf6qUNUwIM2LcpUAFRMdJ7IHrSret9f6haBAlrBuqrutwqGo2ARjVsHiHQfDoHsKkDoLQLBMrLEm8T7LcCPsw9vb895x1BTY2gdAK9HYodi6lrO1oMP12iLY/zmR53QpK%2BAvD5h12PC3Z6Hd429X7srGewinnVU8R8CwQj/X8S0uwAyxb98MFOKwcl6VH8e2HBfVq99S7ohfuI39OD5GEMgfPGB2jUGP2/tI/B5bJ2xMnck5zSt2H6IJCmEz5nygZjVFwHVJeN2vBQqCIYo5FQxgs%2BF1MNn9TsAtAlkYAAyhyQslYxHjBF8r9QbPjDkoVo4bADgkDoGq1hIwYzHaniOpKNYkDshC4JWczIRgmDoHQE1CUDRCM/oKELlXnuVBi5ODMzkuBZdGAV4iUEbQOQB4SWIvFWQABK4y3U9jj0Qa0G2k8QCT5JGyrWYhp4z1F7P0Q8/mt0uxAnzp0%2Bw3vFn1PsM6Yl%2BowWCvWR88bYAH5p%2Bk2Ahn0fOTPPi3yq5WsLNWYNsNDkMrVxF6PP3RhVoIDj4ddFqTQkYUeLEf97z2lBL1r3qW1DVxu7cntxwbkqBagE6srYRgVL4ehAP6ESoR/IjAEZpZC1PAn5hv31sFo9vrDckwHEC1DvUv3mA7BfBD0WDaFl2IinV5hvwC3JW7HXG5Cv1QI/wtTAMrROGwBKQrRw2nSiG52TyQIdBQItWv1sAwNsCwJhRwKqh2XCnUnoI2V2S93YPYLV0SHrVnjuAXi2mSXSjd2EMFy4JqCICID8GEBAGRD/TkMnA6D2GIEnFFjkFIDkFQH2g0IOypFTTkCVD8GSBYH7GEDkF2DnWmwsHjDyRREqC4I4IcJFx92mQlyl3D1l1SAAHkAAFXyJGKZatdwjkLw3wxuXofoSoFZbwv4S8E0G/b/bYc/BQbkW4TwYAR2AYM/aEBQZ4c1WwQSfvSA6tGAnsXDEzTIHVAyecHzIwP2PhcycScIgYRIcsGGeAuogWcg/I7A%2BYXAyZVSa7QlCoziao6GOowSCyS/PoZo4KNo%2BSDo5sNSVAzvFgxgmTRJBgzZfQGQNmfpGQfsGQcgNgWQXQQ49AWQErfBRwcQSQS4E4egQ4ogE4nYtmJqEAfsXQZgWQAAFkOOOJkFOPIHOJkEOOkM%2BKeIBJ2PIDgFgBQFFj8D6QsEoGoHhMRNIBAGAGECsCkN1yIHIElzMAsGkIgA1GePIAdmhGSFkAePIFFl0M8LYCYCpMhPIBwCBw4ECDJMIA6HnnsGkJZPxGwFQBzGkBkBpNXT2MBKMR8GeN2PYE4DJiEAaA1GkMgDZmPEM35JEl2AbG4HwXoCVHMk8PmFBJuKkAYF2NkAOKOLJOBLxAAA4owRIoxvjDcmtMw/hbgmpBIIB8ATVgt5hr9NgDsETAgAy2EjBLiBB%2BBHjZTXj3jPi9jfibSWTgTQSQBwS4yviZAeBDiZSowlRJwlR%2BwlRSzvj6BvjdBiyeB7SlQUzAS0zyAITTi2YYTkAQBxBJCcxkTLx/A0SGByAjp/SLTszrT/iGzZAFh6AjA/1iAkAjAHSnSXS3TMQ61po2BvTYzITWZyBrCcB0TrRszkyZT5geBJweBvieArz%2Bx7T%2BwABOC8pUegeYess42QdMzM7cy0mQE018oE98psrMqeO6Y474oAA */
/**********************************************/

#include <cstddef>
#include <functional>
#include <type_traits>
#include <variant>
#include "ARDL/Util/Overloaded.hpp"
#include <iostream>

// Basic constexpr array implementation
template <typename T, std::size_t n>
class constexpr_array
{
public:
    constexpr constexpr_array()
    {
    }

    constexpr constexpr_array(constexpr_array const & other)
    {
        for (auto i = std::size_t{0}; i != n; ++i)
        {
            a[i] = other.a[i];
        }
    }

    constexpr constexpr_array & operator=(constexpr_array const & other)
    {
        for (auto i = std::size_t{0}; i != n; ++i)
        {
            a[i] = other.a[i];
        }
        return *this;
    }

    constexpr auto operator[](std::size_t i) const
        -> T const &
    {
        return a[i];
    }

    constexpr auto operator[](std::size_t i)
        -> T &
    {
        return a[i];
    }
private:
    T a[n] = {};
};

// constexpr helper to transform multiple indices to a single
// index and back again
template <std::size_t... dims>
struct multi_array_index
{
    // factor applied to the index of this dimension
    static constexpr auto indexFactor(std::size_t dimIndex)
    {
        auto ret = std::size_t{1};
        auto dimIter = std::size_t{0};
        ((dimIter++ < dimIndex && (ret *= dims)),...);
        return ret; 
    }

    // combine muliple indices to a single index
    template <typename... Indices>
    static constexpr auto combine(Indices... indices) noexcept
    {
        auto ret = std::size_t{0};
        auto dimIter = std::size_t{0};
        ((ret += indexFactor(dimIter++)*indices),...);
        return ret;
    }

    // split single index into seperated index
    // this is an inverse function of combine
    static constexpr auto split(std::size_t combined) noexcept
    {
        constexpr_array<std::size_t, sizeof...(dims)> indices;
        auto dimIter = std::size_t{0};
        for (auto dimIter=std::size_t{0}; dimIter != sizeof...(dims); ++dimIter)
            indices[dimIter] = combined%indexFactor(dimIter+1)/indexFactor(dimIter);
        return indices;
    }

    // total number of all indices
    static constexpr auto number_of_entries() noexcept
    {
        return indexFactor(sizeof...(dims));
    }
};

#ifdef UNCHECKED_GET
#error UNCHECKED_GET already defined
#else
#ifdef __GNUC__
#define UNCHECKED_GET std::__detail::__variant::__get
#else
#define UNCHECKED_GET std::get
#endif
#endif

// implemenation of my_visit
template <
    typename TestCi,
    typename IsVis>
struct my_visit_impl;

template <
    typename TestCi,
    std::size_t... vis>
struct my_visit_impl<TestCi, std::index_sequence<vis...>>
{
template <
    typename Visitor, 
    typename... Variants>
static inline decltype(auto) call(
    std::size_t ci,
    Visitor && visitor,
    Variants &&... variants)
{
    using MAI = multi_array_index<std::variant_size_v<std::decay_t<Variants>>...>;

    constexpr auto testCi = TestCi::value;

    if constexpr (testCi < MAI::number_of_entries())
    {
        if (testCi != ci)
            return my_visit_impl<
              std::integral_constant<std::size_t, testCi + 1>,
              std::index_sequence<vis...>>::call(
                ci,
                std::forward<Visitor>(visitor),
                std::forward<Variants>(variants)...);
    }
    return std::invoke(
        std::forward<Visitor>(visitor),
        UNCHECKED_GET<MAI::split(testCi)[vis]>(std::forward<Variants>(variants))...);
}
};

#undef UNCHECKED_GET

// alternative implemenation to std::visit
template <typename Visitor, typename... Variants>
inline decltype(auto) my_visit(Visitor && visitor, Variants &&... variants)
{
    if ((variants.valueless_by_exception() ||...))
        throw std::bad_variant_access();
    using MAI = multi_array_index<std::variant_size_v<std::remove_reference_t<Variants>>...>;
    auto const ci = MAI::combine(variants.index()...);
    return my_visit_impl<
      std::integral_constant<std::size_t, 0>,
      decltype(std::make_index_sequence<sizeof...(variants)>())>::call(
        ci,
        std::forward<Visitor>(visitor),
        std::forward<Variants>(variants)...);
}

/**********************************************/
/* End of my_visit implementation             */
/**********************************************/


template <size_t N, typename R, typename Variant, typename Visitor>
constexpr R visit_nt(Variant &&var, Visitor &&vis) {
  if constexpr (N == 0) {
    if (N == var.index()) {
        // If this check isnt there the compiler will generate
        // exception code, this stops that
      return std::forward<Visitor>(vis)(
          std::get<N>(std::forward<Variant>(var)));
    }
  } else {
    if (var.index() == N) {
      return std::forward<Visitor>(vis)(
          std::get<N>(std::forward<Variant>(var)));
    }
    return visit_nt<N - 1, R>(std::forward<Variant>(var),
                              std::forward<Visitor>(vis));
                              
  }
  while( true ) {} 
}

template <class... Args, typename Visitor, typename... Visitors>
constexpr decltype(auto) visit_nt(
    std::variant<Args...> const &var, Visitor &&vis, Visitors &&... visitors) {
  auto ol =
      overloaded(std::forward<Visitor>(vis), std::forward<Visitors>(visitors)...);
  using result_t = decltype(std::invoke(std::move(ol), std::get<0>(var)));

  static_assert(sizeof...(Args) > 0);
  return visit_nt<sizeof...(Args) - 1, result_t>(var, std::move(ol));
}

template <class... Args, typename Visitor, typename... Visitors>
constexpr decltype(auto) visit_nt(std::variant<Args...> &var,
                                                Visitor &&vis,
                                                Visitors &&... visitors) {
  auto ol =
      overloaded{std::forward<Visitor>(vis), std::forward<Visitors>(visitors)...};
  using result_t = decltype(std::invoke(std::move(ol), std::get<0>(var)));

  static_assert(sizeof...(Args) > 0);
  return visit_nt<sizeof...(Args) - 1, result_t>(var, std::move(ol));
}

template <class... Args, typename Visitor, typename... Visitors>
constexpr decltype(auto) visit_nt(std::variant<Args...> &&var,
                                                Visitor &&vis,
                                                Visitors &&... visitors) {
  auto ol =
      overloaded{std::forward<Visitor>(vis), std::forward<Visitors>(visitors)...};
  using result_t =
      decltype(std::invoke(std::move(ol), std::move(std::get<0>(var))));

  static_assert(sizeof...(Args) > 0);
  return visit_nt<sizeof...(Args) - 1, result_t>(std::move(var), std::move(ol));
}

template <typename Value, typename... Visitors>
inline constexpr bool is_visitable_v = (std::is_invocable_v<Visitors, Value> or
                                        ...);

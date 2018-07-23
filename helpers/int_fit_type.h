#pragma once

// from: https://stackoverflow.com/questions/7038797/automatically-pick-a-variable-type-big-enough-to-hold-a-specified-number

#include <limits>
#include <type_traits>

template<class T, class U =
typename std::conditional<std::is_signed<T>::value,
    std::intmax_t,
    std::uintmax_t
>::type>
constexpr bool is_in_range(U x) {
    return (x >= std::numeric_limits<T>::min())
           && (x <= std::numeric_limits<T>::max());
}

template<std::intmax_t x>
using int_fit_type =
typename std::conditional<is_in_range<std::int8_t>(x),
    std::int8_t,
    typename std::conditional<is_in_range<std::int16_t>(x),
        std::int16_t,
        typename std::conditional<is_in_range<std::int32_t>(x),
            std::int32_t,
            typename std::enable_if<is_in_range<std::int64_t>(x), std::int64_t>::type
        >::type
    >::type
>::type;

template<std::uintmax_t x>
using uint_fit_type =
typename std::conditional<is_in_range<std::uint8_t>(x),
    std::uint8_t,
    typename std::conditional<is_in_range<std::uint16_t>(x),
        std::uint16_t,
        typename std::conditional<is_in_range<std::uint32_t>(x),
            std::uint32_t,
            typename std::enable_if<is_in_range<std::uint64_t>(x), std::uint64_t>::type
        >::type
    >::type
>::type;


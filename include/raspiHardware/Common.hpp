#ifndef ROBOT_INTERFACE2_RASPIHARDWARE_COMMON_HPP
#define ROBOT_INTERFACE2_RASPIHARDWARE_COMMON_HPP
#include <cstring>
#include <cstdint>
#include <type_traits>
#include "robot_interface2/span.hpp"
using tcb::span; // This is here to make it easier for us to transition to c++20 span


constexpr uint32_t Peripheral_Base = 0xFE000000;
constexpr uint32_t Page_Size = 4096;

// bit_cast implementation taken from https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
typename std::enable_if_t<
    sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> && std::is_trivially_copyable_v<To>, To>
// constexpr support needs compiler magic
bit_cast(const From &src) noexcept
{
    static_assert(std::is_trivially_constructible_v<To>,
                  "This implementation additionally requires destination type to be trivially constructible");

    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}

#endif // ROBOT_INTERFACE2_RASPIHARDWARE_COMMON_HPP

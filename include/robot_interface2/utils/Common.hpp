#ifndef ROBOT_INTERFACE2_UTILS_COMMON_HPP
#define ROBOT_INTERFACE2_UTILS_COMMON_HPP
#include <array>
#include <cstring>
template <typename T> inline std::array<uint8_t, sizeof(T)> BytesFrom(T input)
{
    std::array<uint8_t, sizeof(T)> temp;
    std::memcpy(temp.data(), &input, sizeof(T));
    return temp;
}

template <typename T> inline T FromBytes(tcb::span<const uint8_t, sizeof(T)> input)
{
    T temp;
    std::memcpy(&temp, input.data(), sizeof(T));
    return temp;
}

#endif // ROBOT_INTERFACE2_UTILS_COMMON_HPP

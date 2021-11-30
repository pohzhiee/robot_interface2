#ifndef ROBOT_INTERFACE2_FLYSKY_COMMON_HPP
#define ROBOT_INTERFACE2_FLYSKY_COMMON_HPP

#include <cstdint>
namespace robot_interface2::Flysky
{
class Value
{
  public:
    explicit Value(uint32_t initial_value = 1500) : raw_data(initial_value){};
    static const uint32_t max_val = 2000;
    static const uint32_t min_val = 1000;
    uint32_t raw_data;
    [[nodiscard]] inline double get_percentage() const
    {
        return (double)(raw_data - min_val) / (max_val - min_val);
    }

    [[nodiscard]] inline bool is_valid() const
    {
        return raw_data < max_val && raw_data > min_val;
    }

    /**
     * Centre the data, assume that the data with +-filter_val is still at centre
     * @param filter_val
     * @return
     */
    [[nodiscard]] inline int32_t centre_with_filter(int32_t filter_val) const
    {
        //        constexpr uint32_t range = max_val - min_val;
        constexpr int32_t centre = (max_val + min_val) / 2;
        if (centre - filter_val < raw_data && centre + filter_val > raw_data)
        {
            return 0;
        }
        return raw_data - centre;
    };
};

enum class SwitchState
{
    Up,
    Middle,
    Down,
    Unknown
};

struct RemoteState
{
    Value Channel1{}; // RHS -> LR
    Value Channel2{}; // RHS -> UpDown
    Value Channel3{}; // LHS -> UpDown
    Value Channel4{}; // LHS -> LR
    Value Channel5{}; // VrA -> Clockwise
    Value Channel6{}; // VrB -> Clockwise
    SwitchState SwitchA{SwitchState::Unknown};
    SwitchState SwitchB{SwitchState::Unknown};
    SwitchState SwitchC{SwitchState::Unknown};
    SwitchState SwitchD{SwitchState::Unknown};
    SwitchState SwitchE{SwitchState::Unknown};
};

} // namespace robot_interface2::Flysky
#endif // ROBOT_INTERFACE2_FLYSKY_COMMON_HPP

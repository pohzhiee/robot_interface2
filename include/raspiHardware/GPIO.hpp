#ifndef ROBOT_INTERFACE2_GPIO_HPP
#define ROBOT_INTERFACE2_GPIO_HPP
#include "Common.hpp"

struct GPIORegisters
{
    volatile uint32_t GPFSEL0 : 32;
    volatile uint32_t GPFSEL1 : 32;
    volatile uint32_t GPFSEL2 : 32;
    volatile uint32_t GPFSEL3 : 32;
    volatile uint32_t GPFSEL4 : 32;
    volatile uint32_t GPFSEL5 : 32;
    uint32_t : 32;
    volatile uint32_t GPSET0 : 32;
    volatile uint32_t GPSET1 : 32;
    uint32_t : 32;
    volatile uint32_t GPCLR0 : 32;
    volatile uint32_t GPCLR1 : 32;
    uint32_t : 32;
    volatile uint32_t GPLEV0 : 32;
    volatile uint32_t GPLEV1 : 32;
    uint32_t : 32;
    volatile uint32_t GPEDS0 : 32;
    volatile uint32_t GPEDS1 : 32;
    uint32_t : 32;
    volatile uint32_t GPREN0 : 32;
    volatile uint32_t GPREN1 : 32;
    uint32_t : 32;
    volatile uint32_t GPFEN0 : 32;
    volatile uint32_t GPFEN1 : 32;
    uint32_t : 32;
    volatile uint32_t GPHEN0 : 32;
    volatile uint32_t GPHEN1 : 32;
    uint32_t : 32;
    volatile uint32_t GPLEN0 : 32;
    volatile uint32_t GPLEN1 : 32;
    uint32_t : 32;
    volatile uint32_t GPAREN0 : 32;
    volatile uint32_t GPAREN1 : 32;
    uint32_t : 32;
    volatile uint32_t GPAFEN0 : 32;
    volatile uint32_t GPAFEN1 : 32;
};
static_assert(sizeof(GPIORegisters) == 144);

enum class GPIO_AlternateFunc : uint8_t
{
    Input = 0b000,
    Output = 0b001,
    AF0 = 0b100,
    AF1 = 0b101,
    AF2 = 0b110,
    AF3 = 0b111,
    AF4 = 0b011,
    AF5 = 0b010
};

constexpr uint32_t GPIO_Offset = 0x200000;
constexpr uint32_t GPIO_Base = Peripheral_Base + GPIO_Offset;

template <unsigned pin_num> void SetAF(uintptr_t gpio_ptr, GPIO_AlternateFunc AF)
{
    constexpr uint32_t gpfsel_reg_index = pin_num / 10;
    constexpr uint32_t addr_offset = 0x04 * gpfsel_reg_index;
    constexpr uint32_t gpfsel_bit_pos = (pin_num % 10) * 3;

    auto *gpfsel_reg = reinterpret_cast<uint32_t *>(gpio_ptr + addr_offset);
    *gpfsel_reg = ((*gpfsel_reg) & (~(0b111 << gpfsel_bit_pos))) | (static_cast<uint8_t>(AF)) << gpfsel_bit_pos;
}

template <unsigned N> class GPIO_Output
{
    static_assert(N < 28);

  public:
    explicit GPIO_Output(uintptr_t gpio_reg_ptr) : gpio_reg_ptr_(reinterpret_cast<GPIORegisters *>(gpio_reg_ptr))
    {
        constexpr uint32_t gpfsel_reg_index = N / 10;
        constexpr uint32_t addr_offset = 0x04 * gpfsel_reg_index;
        constexpr uint32_t gpfsel_bit_pos = (N % 10) * 3;

        auto *gpfsel_reg = reinterpret_cast<uint32_t *>(gpio_reg_ptr + addr_offset);
        *gpfsel_reg = ((*gpfsel_reg) & (~(0b111 << gpfsel_bit_pos))) | static_cast<uint8_t>(GPIO_AlternateFunc::Output)
                                                                           << gpfsel_bit_pos;
    }

    void Toggle()
    {
        bool is_set = gpio_reg_ptr_->GPLEV0 & (0b1 << N);
        if (is_set)
            gpio_reg_ptr_->GPCLR0 = 0b1 << N;
        else
            gpio_reg_ptr_->GPSET0 = 0b1 << N;
    }

    void Set()
    {
        gpio_reg_ptr_->GPSET0 = 0b1 << N;
    }

    void Clear()
    {
        gpio_reg_ptr_->GPCLR0 = 0b1 << N;
    }

    explicit operator bool()
    {
        return gpio_reg_ptr_->GPLEV0 & (0b1 << N);
    }

  private:
    GPIORegisters *const gpio_reg_ptr_;
};

#endif // ROBOT_INTERFACE2_GPIO_HPP

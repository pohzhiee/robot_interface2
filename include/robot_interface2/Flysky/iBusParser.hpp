#ifndef ROBOTINTERFACE2_FLYSKY_IBUSPARSER_HPP
#define ROBOTINTERFACE2_FLYSKY_IBUSPARSER_HPP

#include "robot_interface2/Flysky/Common.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include "robot_interface2/span.hpp"
#include <deque>
#include <functional>
#include <vector>

namespace robot_interface2::Flysky
{
std::ostream &operator<<(std::ostream &out, const SwitchState &state);

class iBusParser
{
  public:
    struct CallbackInfo
    {
        tcb::span<uint8_t> bufferSinceLastCb;
        tcb::span<uint8_t> dataBuffer;
        uint32_t correctBufferStartIndex;
        Value errorRate;
        Value SNR;
    };
    using CallbackFunc = std::function<void(RemoteState, CallbackInfo)>;
    void AddByte(uint8_t byte);
    void AddCallback(const CallbackFunc &func);

  private:
    enum class State
    {
        PendingHeader1,
        PendingHeader2,
        PendingData,
        PendingChecksum1,
        PendingChecksum2
    };
    std::vector<uint8_t> dataStore_{};
    std::vector<CallbackFunc> callbacks_{};
    State currentState_{State::PendingHeader1};
    uint32_t bytesLeftToChecksum_{0};
    uint32_t currentDataStartIndex_{0};
    std::deque<std::vector<uint8_t>> dataStoreBuffer_{};
};

} // namespace robot_interface2::Flysky
#endif // ROBOTINTERFACE2_FLYSKY_IBUSPARSER_HPP

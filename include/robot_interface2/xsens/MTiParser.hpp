#ifndef ROBOT_INTERFACE2_MTIPARSER_HPP
#define ROBOT_INTERFACE2_MTIPARSER_HPP
#include "robot_interface2/span.hpp"
#include <functional>
#include <deque>
#include <vector>
#include <xstypes/xsmessage.h>

namespace robot_interface2{

class MTiParser
{
public:
    struct CallbackInfo
    {
        tcb::span<uint8_t> buffer_since_last_callback;
        uint32_t correct_buffer_start_index;
    };
    using CallbackFunc = std::function<void(const XsMessage&, CallbackInfo)>;
    void AddByte(uint8_t byte);
    void AddCallback(CallbackFunc func);
    void ClearCallbacks();

private:
    enum class State
    {
        PendingHeader,
        PendingId,
        PendingMID,
        PendingLength,
        PendingChecksum
    };
    std::vector<uint8_t> dataStore_{};
    std::vector<CallbackFunc> callbacks_{};
    State currentState_{State::PendingHeader};
    uint8_t bytesLeftToChecksum_{0};
    uint32_t currentDataStartIndex_{0};
    uint32_t messageTotalSize_{0};
    std::deque<std::vector<uint8_t>> dataStoreBuffer_{};
};
}
#endif //ROBOT_INTERFACE2_MTIPARSER_HPP

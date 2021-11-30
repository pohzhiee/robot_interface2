#include "robot_interface2/Flysky/iBusParser.hpp"
#include <cassert>
#include <cstring>
#include <iostream>

namespace robot_interface2::Flysky
{

SwitchState GetSwitchState(uint16_t val)
{
    switch (val)
    {
    case 1000:
        return SwitchState::Up;
    case 1500:
        return SwitchState::Middle;
    case 2000:
        return SwitchState::Down;
    default:
        return SwitchState::Unknown;
    }
}

bool IsChecksumOk(tcb::span<uint8_t> data)
{
    assert(data.size() == 32);
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++)
    {
        checksum -= data[i];
    }
    uint16_t oriChecksum = 0;
    std::memcpy(&oriChecksum, &data[30], 2);
    return checksum == oriChecksum;
}

void iBusParser::AddByte(uint8_t byte)
{
    dataStore_.push_back(byte);
    // The only time this switch statement doesn't return is when Checksum2 byte is received
    switch (currentState_)
    {
    case State::PendingHeader1:
        if (byte == 0x20)
            currentState_ = State::PendingHeader2;

        return;
    case State::PendingHeader2:
        if (byte == 0x40)
        {
            currentState_ = State::PendingData;
            currentDataStartIndex_ = dataStore_.size() - 2;
            bytesLeftToChecksum_ = 28;
        }
        else
        {
            currentState_ = State::PendingHeader1;
        }
        return;
    case State::PendingData:
        assert(bytesLeftToChecksum_ > 0);
        bytesLeftToChecksum_--;
        if (bytesLeftToChecksum_ == 0)
            currentState_ = State::PendingChecksum1;
        return;
    case State::PendingChecksum1:
        currentState_ = State::PendingChecksum2;
        return;
    case State::PendingChecksum2:
        currentState_ = State::PendingHeader1;
        break;
    }

    auto dataSpan = tcb::span<uint8_t>(&dataStore_[currentDataStartIndex_], 32);
    assert(dataSpan[0] == 0x20);
    assert(dataSpan[1] == 0x40);
    bool checksumOk = IsChecksumOk(dataSpan);
    if (checksumOk)
    {
        auto dataStoreRef = dataStoreBuffer_.emplace_back(dataStore_);
        if (dataStoreBuffer_.size() > 20)
        {
            dataStoreBuffer_.pop_front();
        }
        std::array<uint16_t, 14> channelValues{};
        std::memcpy(channelValues.data(), dataSpan.data() + 2, 28);
        RemoteState state{
            Value(channelValues[0]),          Value(channelValues[1]),          Value(channelValues[2]),
            Value(channelValues[3]),          Value(channelValues[4]),          Value(channelValues[5]),
            GetSwitchState(channelValues[6]), GetSwitchState(channelValues[7]), GetSwitchState(channelValues[8]),
            GetSwitchState(channelValues[9]), GetSwitchState(channelValues[10])};

        CallbackInfo info{tcb::span<uint8_t>(dataStoreRef.data(), dataStoreRef.size()), dataSpan,
                          currentDataStartIndex_, Value(channelValues[12]), Value(channelValues[13])};
        for (auto &callback : callbacks_)
        {
            callback(state, info);
        }
        dataStore_.clear();
        dataStore_.reserve(64);
    }
    else
    {
        std::cerr << "Flysky Checksum not ok" << std::endl;
    }
}

void iBusParser::AddCallback(const CallbackFunc &func)
{
    callbacks_.push_back(func);
}

std::ostream &operator<<(std::ostream &out, const SwitchState &state)
{
    switch (state)
    {
    case SwitchState::Unknown:
        out << "SwitchState::Unknown";
        break;
    case SwitchState::Up:
        out << "SwitchState::Up";
        break;
    case SwitchState::Down:
        out << "SwitchState::Down";
        break;
    case SwitchState::Middle:
        out << "SwitchState::Middle:";
        break;
    }
    return out;
}
} // namespace RobotInterface::Flysky
#include "robot_interface2/xsens/MTiParser.hpp"
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsmessage.h>
#include <iostream>

namespace robot_interface2{

void MTiParser::AddByte(uint8_t byte)
{
    dataStore_.push_back(byte);
    switch (currentState_)
    {
    case State::PendingHeader:
        if (byte == 0xFA)
        {
            currentState_ = State::PendingId;
        }
        break;
    case State::PendingId:
        if (byte == 0xFF)
        {
            currentState_ = State::PendingMID;
        }
        else
        {
            currentState_ = State::PendingHeader;
        }
        break;
    case State::PendingMID:
        if (byte == XMID_MtData2)
        {
            currentState_ = State::PendingLength;
        }
        else
        {
            currentState_ = State::PendingHeader;
        }
        break;
    case State::PendingLength:
        bytesLeftToChecksum_ = byte;
        messageTotalSize_ = byte + 5;
        if (dataStore_.size() < 4)
        {
            std::cerr << "MTi Data parser buffer issue, when length byte received there is less than 4 items in buffer"
                      << std::endl;
        }
        currentDataStartIndex_ = dataStore_.size() - 4;
        currentState_ = State::PendingChecksum;
        return;
    case State::PendingChecksum:
        if(bytesLeftToChecksum_ != 0){
            bytesLeftToChecksum_--;
            return;
        }
        currentState_ = State::PendingHeader;
        [&](){
            assert(messageTotalSize_ < 300); // check for sane message size
            XsMessage msg(&dataStore_[currentDataStartIndex_], messageTotalSize_);

            bool checksumOk = XsMessage_isChecksumOk(&msg) != 0;
            if (checksumOk)
            {
                CallbackInfo info;
                auto dataRef = dataStoreBuffer_.emplace_back(dataStore_);
                if (dataStoreBuffer_.size() > 20)
                {
                    dataStoreBuffer_.pop_front();
                }
                info.buffer_since_last_callback = tcb::span<uint8_t>(dataRef.data(), dataRef.size());
                info.correct_buffer_start_index = currentDataStartIndex_;
                for (auto &callback : callbacks_)
                {
                    callback(msg, info);
                }
                dataStore_ = std::vector<uint8_t>();
                dataStore_.reserve(256);
                messageTotalSize_ = 0;
            }
            else
            {
                std::cerr << "Checksum not ok" << std::endl;
            }
        }();
    }
}
void MTiParser::AddCallback(MTiParser::CallbackFunc func)
{
    callbacks_.push_back(func);
}
void MTiParser::ClearCallbacks()
{
    callbacks_ = {};
}
}
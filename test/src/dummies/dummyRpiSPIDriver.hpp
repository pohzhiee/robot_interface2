#ifndef ROBOTINTERFACE_TEST_DUMMYRPISPIDRIVER_HPP
#define ROBOTINTERFACE_TEST_DUMMYRPISPIDRIVER_HPP
#include "raspiHardware/SPI.hpp"
#include <map>
#include <utility>
#include <vector>

namespace robot_interface2_test
{
class DummyRpiSPIDriver : public RpiSPIDriver
{
    struct TransceiveDataInput
    {
        uint8_t spiNum;
        tcb::span<const uint8_t> txBuf;
        tcb::span<uint8_t> rxBuf;
    };
    struct TranceivedData
    {
        uint8_t spiNum;
        std::vector<uint8_t> txBuf;
        std::vector<uint8_t> rxBuf;
    };

  public:
    void AddTransceiveData(uint8_t spiNum, tcb::span<const uint8_t> txBuf, tcb::span<uint8_t> rxBuf) override
    {
        if (spiNum > 6)
            throw std::runtime_error("Invalid SPI number");
        transceiveQueueCount_++;
        transceiveQueue_.emplace_back(TransceiveDataInput{.spiNum = spiNum, .txBuf = txBuf, .rxBuf = rxBuf});
    };
    void Transceive() override
    {
        auto contains = [](auto a, auto val) {
            auto it = a.find(val);
            // Check if element exists in map or not
            return static_cast<bool>(it != a.end());
        };
        for (const auto &transceiveData : transceiveQueue_)
        {
            if (contains(rxBytes_, transceiveData.spiNum))
            {
                const auto &rxByteVec = rxBytes_[transceiveData.spiNum];
                for (size_t i = 0; i < transceiveData.rxBuf.size() && i < rxByteVec.size(); i++)
                {
                    transceiveData.rxBuf[i] = rxByteVec[i];
                }
            }
            transceiveFinish_.emplace_back(
                TranceivedData{.spiNum = transceiveData.spiNum,
                               .txBuf = {transceiveData.txBuf.begin(), transceiveData.txBuf.end()},
                               .rxBuf = {transceiveData.rxBuf.begin(), transceiveData.rxBuf.end()}});
        }
        transceiveQueue_.clear();
        totalTransceiveCount_ += transceiveQueueCount_;
        transceiveQueueCount_ = 0;
    };

    void SetRxBytes(std::vector<uint8_t> rxBytes, uint8_t spiNum)
    {
        rxBytes_[spiNum] = std::move(rxBytes);
    };

    [[nodiscard]] uint32_t GetTransceiveQueueCount() const
    {
        return transceiveQueueCount_;
    }
    [[nodiscard]] uint32_t GetTotalTransceiveCount() const
    {
        return totalTransceiveCount_;
    }
    [[nodiscard]] auto GetTransceivedData() const
    {
        return transceiveFinish_;
    }

  private:
    uint32_t transceiveQueueCount_{0};
    uint32_t totalTransceiveCount_{0};
    std::map<uint8_t, std::vector<uint8_t>> rxBytes_{};
    std::vector<TransceiveDataInput> transceiveQueue_{};
    std::vector<TranceivedData> transceiveFinish_{};
};
} // namespace RobotInterface
#endif
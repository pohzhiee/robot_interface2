#include "raspiHardware/SimultaneousSPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include <algorithm>

SimultaneousSPI::SimultaneousSPI(SPISettings spiSettings, uintptr_t gpioPtr, uintptr_t spiBasePtr)
    : settings_(spiSettings), gpioBasePtr_(gpioPtr), spiBasePtr_(spiBasePtr)
{
    CSDefaults_.at(0) = spiSettings.SPI0_CSDefault;
    CSDefaults_.at(3) = spiSettings.SPI3_CSDefault;
    CSDefaults_.at(6) = spiSettings.SPI6_CSDefault;
}
void SimultaneousSPI::AddTransceiveData(uint8_t spiNum, span<const uint8_t> txBuf, span<uint8_t> rxBuf)
{
    CheckAndInitialiseSPI(spiNum);
    transceiveIndex.emplace_back(spiNum);
    if (rxBuf.size() > txBuf.size())
        throw std::runtime_error("Rx buffer cannot be bigger than tx buffer");
    txBufs_.at(spiNum) = txBuf;
    rxBufs_.at(spiNum) = rxBuf;
}
void SimultaneousSPI::Transceive()
{
    std::array<bool, 7> txDone{};
    std::array<bool, 7> rxDone{};
    std::array<uint32_t, 7> txCounts{};
    std::array<uint32_t, 7> rxCounts{};
    // Set the transfer active bit to initiate transfer
    for (uint8_t index : transceiveIndex)
    {
        spiPtrs_.at(index)->CS |= 0b1u << 7u;
    }
    bool setTxBufAllDone = false;
    bool rxReadAllDone = false;
    while (!setTxBufAllDone || !rxReadAllDone)
    {
        // Transmit and receive a single byte of data if possible
        for (uint8_t index : transceiveIndex)
        {
            txDone.at(index) = txCounts.at(index) == txBufs_.at(index).size();
            rxDone.at(index) = rxCounts.at(index) == rxBufs_.at(index).size();

            if (!rxDone.at(index))
            {
                volatile uint32_t status = spiPtrs_.at(index)->CS;
                if ((status & (0b1u << 17u)) != 0u) // READ RXD bit
                {
                    volatile uint32_t data = spiPtrs_.at(index)->FIFO;
                    rxBufs_.at(index)[rxCounts.at(index)] = static_cast<uint8_t>(data);
                    rxCounts.at(index)++;
                }
            }

            if (!txDone.at(index))
            {
                volatile uint32_t status = spiPtrs_.at(index)->CS;
                if ((status & (0b1u << 18u)) != 0u) // READ TXD bit
                {
                    uint32_t data = txBufs_.at(index)[txCounts.at(index)];
                    spiPtrs_.at(index)->FIFO = data;
                    txCounts.at(index)++;
                }
            }
        }

        // Check whether all data is fully transmitted and received
        rxReadAllDone = std::all_of(std::begin(transceiveIndex), std::end(transceiveIndex),
                                    [&](uint8_t index) { return rxDone[index]; });
        setTxBufAllDone = std::all_of(std::begin(transceiveIndex), std::end(transceiveIndex),
                                      [&](uint8_t index) { return txDone[index]; });
    }

    // Wait for all transmission to finish
    // (e.g. tx buffer might be able to fit more but the data is not transmitted over the line yet)
    bool allTransmitted = false;
    while (!allTransmitted)
    {
        allTransmitted =
            std::all_of(std::begin(transceiveIndex), std::end(transceiveIndex),
                        [&](uint8_t index) -> bool { return (spiPtrs_.at(index)->CS & (0b1u << 16u)) != 0u; });
    }

    // Reset all the SPI peripherals (mostly to disable transmit and reset FIFO)
    for (uint8_t index : transceiveIndex)
    {
        spiPtrs_.at(index)->CS = CSDefaults_.at(index);
    }
    transceiveIndex.clear();
}
void SimultaneousSPI::CheckAndInitialiseSPI(uint8_t spiNum)
{
    switch (spiNum)
    {
    case 0:
        if (spiPtrs_.at(spiNum) == nullptr)
        {
            // Set AF0 for GPIO 8,9,10,11
            SetAF<8>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF0);
            SetAF<9>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF0);
            SetAF<10>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF0);
            SetAF<11>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF0);
            spiPtrs_.at(spiNum) = GetSPI<0>(spiBasePtr_);
            // supposedly stops inter-byte gap on SPI0, taken from pigpio
            // see https://github.com/joan2937/pigpio/blob/accd69d2fdd3404a0c56416912d87b2619d58563/pigpio.c#L4528
            spiPtrs_.at(spiNum)->DLEN = 2; /* undocumented, stops inter-byte gap */
            spiPtrs_.at(spiNum)->CS = settings_.SPI0_CSDefault;
            spiPtrs_.at(spiNum)->CLK = settings_.SPI0_CLK;
        }
        break;
    case 6:
        if (spiPtrs_.at(spiNum) == nullptr)
        {
            // Set AF3 for GPIO 18,19,20,21
            SetAF<18>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF3);
            SetAF<19>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF3);
            SetAF<20>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF3);
            SetAF<21>(reinterpret_cast<uintptr_t>(gpioBasePtr_), GPIO_AlternateFunc::AF3);
            spiPtrs_.at(spiNum) = GetSPI<6>(spiBasePtr_); // supposedly stops inter-byte gap on SPI0, taken from pigpio
            // see https://github.com/joan2937/pigpio/blob/accd69d2fdd3404a0c56416912d87b2619d58563/pigpio.c#L4528
            spiPtrs_.at(spiNum)->DLEN = 2; /* undocumented, stops inter-byte gap */
            spiPtrs_.at(spiNum)->CS = settings_.SPI6_CSDefault;
            spiPtrs_.at(spiNum)->CLK = settings_.SPI6_CLK;
        }
        break;
    default:
        throw std::runtime_error("Invalid SPI number given");
    }
}
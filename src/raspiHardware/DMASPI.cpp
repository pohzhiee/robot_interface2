#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/UncachedMem.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::chrono;

namespace robot_interface2
{
void DMASPI::InitialiseSPI(uint8_t spiNum, uintptr_t gpioMmapPtr, uintptr_t spiMmapPtr, uint32_t CLK)
{
    switch (spiNum)
    {
    case 0:
        // Set AF0 for GPIO 8,9,10,11
        SetAF<8>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
        SetAF<9>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
        SetAF<10>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
        SetAF<11>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
        break;
    case 6:
        // Set AF3 for GPIO 18,19,20,21
        SetAF<18>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
        SetAF<19>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
        SetAF<20>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
        SetAF<21>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
        break;
    default:
        throw std::runtime_error("Invalid SPI number given");
    }
    auto spi = GetSPI(spiMmapPtr, spiNum);
    // Set CLK
    spi->CLK = CLK;
    // Clear FIFO
    spi->CS = 0b11u << 4u;
    // Set DMAEN, set ADCS (automatically de assert CS, used by DMA), Use 32bit FIFO write
    spi->CS = 0b1 << 8 | 0b1 << 11 | 0b1 << 25;
    spi->DLEN = sizeof(CommandMessage);
}

void DMASPI::SetupControlBlocks(uint8_t spiNum, uint16_t dma_len, uintptr_t txPhysAddr, uintptr_t rxPhysAddr,
                                volatile DMAControlBlock &txConBlock, volatile DMAControlBlock &rxConBlock)
{
    uint8_t rxNum;
    uint8_t txNum;
    auto spiPhysAddr = 0x7e204000 + spiNum * 0x200;
    switch (spiNum)
    {
    case 0:
        rxNum = 7;
        txNum = 6;
        break;
    case 6:
        rxNum = 27;
        txNum = 23;
        break;
    default:
        throw std::runtime_error("Wrong spi num for setting up DMA control blocks");
    }
    DMATransferInformation ti = {.DestAddrIncrement = false,
                                 .DestTransferWidth = DMATransferWidth::Width32,
                                 .DestWriteUseDREQ = true,
                                 .SrcAddrIncrement = true,
                                 .SrcTransferWidth = DMATransferWidth::Width32,
                                 .SrcReadUseDREQ = false,
                                 .PeriphMapNum = txNum,
                                 .WaitCycle = 30};
    uint32_t tempTi;
    std::memcpy(&tempTi, &ti, sizeof(uint32_t));
    txConBlock.TI = tempTi;
    txConBlock.SourceAddr = static_cast<uint32_t>(txPhysAddr);
    txConBlock.DestAddr = static_cast<uint32_t>(spiPhysAddr + 0x04);
    txConBlock.TxLen = (dma_len + 1) * 4;
    txConBlock.Stride = 0;
    txConBlock.NextConBlkAddr = 0;

    auto ti2 = DMATransferInformation{.DestAddrIncrement = true,
                                      .DestTransferWidth = DMATransferWidth::Width32,
                                      .DestWriteUseDREQ = false,
                                      .SrcAddrIncrement = false,
                                      .SrcTransferWidth = DMATransferWidth::Width32,
                                      .SrcReadUseDREQ = true,
                                      .PeriphMapNum = rxNum,
                                      .WaitCycle = 25};
    uint32_t tempTi2;
    std::memcpy(&tempTi2, &ti2, sizeof(uint32_t));
    rxConBlock.TI = tempTi2;
    rxConBlock.SourceAddr = static_cast<uint32_t>(spiPhysAddr + 0x04),
    rxConBlock.DestAddr = static_cast<uint32_t>(rxPhysAddr);
    rxConBlock.TxLen = static_cast<uint32_t>(dma_len * 4);
    rxConBlock.Stride = 0;
    rxConBlock.NextConBlkAddr = 0;
}

DMASPI::DMASPI(std::array<DMASPISetting, 2> spiSettings, uintptr_t gpioMmapPtr, uintptr_t dmaMmapPtr,
               uintptr_t spiMmapPtr)
    : spiSettings_(spiSettings), gpioMmapPtr_(gpioMmapPtr), dmaMmapPtr_(dmaMmapPtr), spiMmapPtr_(spiMmapPtr)
{
    constexpr auto dmaLen = sizeof(CommandMessage) / 4;
    for (int i = 0; i < 2; i++)
    {
        const auto spiNum = i * 6;
        conBlockMemBlock_.at(i) = UncachedMemBlock_alloc(Page_Size);
        auto &conBlockMemBlock = conBlockMemBlock_.at(i);
        auto rxDataMemBlock = UncachedMemBlock_alloc(Page_Size);
        auto txDataMemBlock = UncachedMemBlock_alloc(Page_Size);
        rxBuffer_.at(i) = span<volatile uint32_t, Page_Size / 4>(
            reinterpret_cast<volatile uint32_t *>(rxDataMemBlock.mem.data()), Page_Size / 4);
        txBuffer_.at(i) = span<volatile uint32_t, Page_Size / 4>(
            reinterpret_cast<volatile uint32_t *>(txDataMemBlock.mem.data()), Page_Size / 4);
        conBlocks_.at(i) = span<volatile DMAControlBlock, Page_Size / sizeof(DMAControlBlock)>(
            reinterpret_cast<volatile DMAControlBlock *>(conBlockMemBlock_.at(i).mem.data()),
            Page_Size / sizeof(DMAControlBlock));
        auto &txDmaConBlock = conBlocks_.at(i)[0];
        auto &rxDmaConBlock = conBlocks_.at(i)[1];
        SetupControlBlocks(spiNum, dmaLen, UncachedMemBlock_to_physical(&txDataMemBlock, &txBuffer_.at(i)[0]),
                           UncachedMemBlock_to_physical(&rxDataMemBlock, &rxBuffer_.at(i)[0]), txDmaConBlock,
                           rxDmaConBlock);
        InitialiseSPI(spiNum, gpioMmapPtr, spiMmapPtr, spiSettings[i].SPI_CLK);

        txBuffer_.at(i)[0] = (sizeof(CommandMessage) << 16) | (0b1 << 7);

        auto txDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(i).txDmaNum);
        auto rxDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(i).rxDmaNum);
        txDmaPtr->CtrlAndStatus |= 0b1 << 31;
        rxDmaPtr->CtrlAndStatus |= 0b1 << 31;

        txDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_.at(i), &txDmaConBlock);
        rxDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_.at(i), &rxDmaConBlock);
    }
}
void DMASPI::AddTransceiveData(uint8_t spiNum, span<const uint8_t> txBuf, span<uint8_t> rxBuf)
{
    if (spiNum != 0 && spiNum != 6)
    {
        throw std::runtime_error("Only SPI0 and SPI6 supported");
    }
    if (txBuf.size() % 4 != 0 || rxBuf.size() % 4 != 0)
    {
        throw std::runtime_error("SPI buf size not multiple of 4");
    }
    // Write tx data to uncached mailbox memory for DMA transfer, store rx data pointer for writing later
    int index = spiNum / 6;
    rxBufs_.at(spiNum) = rxBuf;
    for (int i = 0; i < txBuf.size() / 4; i++)
    {
        uint32_t temp = static_cast<uint32_t>(txBuf[i * 4]) | (static_cast<uint32_t>(txBuf[i * 4 + 1]) << 8) |
                        (static_cast<uint32_t>(txBuf[i * 4 + 2]) << 16) |
                        (static_cast<uint32_t>(txBuf[i * 4 + 3]) << 24);
        txBuffer_.at(index)[i + 1] = temp;
    }
}
void DMASPI::Transceive()
{
    GPIO_Output<6> pin6(gpioMmapPtr_);
    pin6.Toggle();
    std::array<SPIRegisters *, 2> spiPtrs = {GetSPI<0>(spiMmapPtr_), GetSPI<6>(spiMmapPtr_)};
    for (int i = 0; i < 2; i++)
    {
        auto txDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(i).txDmaNum);
        auto rxDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(i).rxDmaNum);

        txDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
        rxDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    }
    auto transmitTime = steady_clock::now();
    std::this_thread::sleep_for(100us);
    for (int i = 0; i < 2; i++)
    {
        const auto spiNum = i * 6;
        // Wait for all transmit to be done (i.e. all data is cleared from Rx buffer)
        // This will be slightly later from the time it takes to fully transmit data on the spi bus
        // This is to allow the DMA to fully transfer all the rx data before we read
        auto rxDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(i).rxDmaNum);
        bool timeOut = false;
        while ((rxDmaPtr->CtrlAndStatus & (0b1 << 1)) == 0)
        {
            std::this_thread::sleep_for(1us);
            auto timeDiffUs = duration_cast<microseconds>(steady_clock::now() - transmitTime).count();
            if (timeDiffUs > 1000)
            {
                timeOut = true;
                break;
            }
        }
        if (timeOut)
        {
            pin6.Toggle();
            break;
        }
        for (int k = 0; k < rxBufs_.at(spiNum).size() / 4; k++)
        {
            uint32_t rxBytes = rxBuffer_.at(i)[k];
            rxBufs_.at(spiNum)[k * 4] = rxBytes & 0xFF;
            rxBufs_.at(spiNum)[k * 4 + 1] = ((rxBytes & (0xFF << 8)) >> 8);
            rxBufs_.at(spiNum)[k * 4 + 2] = ((rxBytes & (0xFF << 16)) >> 16);
            rxBufs_.at(spiNum)[k * 4 + 3] = ((rxBytes & (0xFF << 24)) >> 24);
        }
        pin6.Toggle();
    }
    for (int i = 0; i < 2; i++)
    {
        // Reset DMA and SPI
        auto txDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(0 + i).txDmaNum);
        auto rxDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(0 + i).rxDmaNum);

        txDmaPtr->CtrlAndStatus |= 0b1 << 31;
        rxDmaPtr->CtrlAndStatus |= 0b1 << 31;

        // Set DMAEN, set ADCS (automatically de assert CS, used by DMA), Use 32bit FIFO write, reset FIFO
        spiPtrs.at(i)->CS = 0b1 << 8 | 0b1 << 11 | 0b1 << 25 | 0b11u << 4u;

        std::this_thread::sleep_for(10us);
        auto &txDmaConBlock = conBlocks_.at(i)[0];
        auto &rxDmaConBlock = conBlocks_.at(i)[1];
        txDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_.at(i), &txDmaConBlock);
        rxDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_.at(i), &rxDmaConBlock);
    }
}

} // namespace robot_interface2
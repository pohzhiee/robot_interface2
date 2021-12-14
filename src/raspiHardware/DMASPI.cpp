#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/UncachedMem.hpp"
#include "robot_interface2/InterfaceData.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

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
    // Clear FIFO and set DMAEN
    spi->CS = 0b11u << 4u;
    spi->CS = 0b1 << 8;
    // Set DMA transfer number of bytes to be same as command message
    spi->DLEN = sizeof(CommandMessage);
}

DMASPI::ControlBlocks DMASPI::SetupControlBlocks(uint8_t spiNum, uint16_t dma_len, uintptr_t txPhysAddr,
                                                 uintptr_t rxPhysAddr)
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
    DMAControlBlock txConBlock{.TI{DMATransferInformation{.DestAddrIncrement = false,
                                                          .DestTransferWidth = DMATransferWidth::Width32,
                                                          .DestWriteUseDREQ = true,
                                                          .SrcAddrIncrement = true,
                                                          .SrcTransferWidth = DMATransferWidth::Width32,
                                                          .SrcReadUseDREQ = false,
                                                          .PeriphMapNum = txNum,
                                                          .WaitCycle = 20}},
                               .SourceAddr = static_cast<uint32_t>(txPhysAddr),
                               .DestAddr = static_cast<uint32_t>(spiPhysAddr + 0x04),
                               .TxLen = static_cast<uint32_t>((dma_len + 1) * 4),
                               .Stride = 0,
                               .NextConBlkAddr = 0};
    DMAControlBlock rxConBlock{.TI{DMATransferInformation{.DestAddrIncrement = true,
                                                          .DestTransferWidth = DMATransferWidth::Width32,
                                                          .DestWriteUseDREQ = false,
                                                          .SrcAddrIncrement = false,
                                                          .SrcTransferWidth = DMATransferWidth::Width32,
                                                          .SrcReadUseDREQ = true,
                                                          .PeriphMapNum = rxNum,
                                                          .WaitCycle = 1}},
                               .SourceAddr = static_cast<uint32_t>(spiPhysAddr + 0x04),
                               .DestAddr = static_cast<uint32_t>(rxPhysAddr),
                               .TxLen = static_cast<uint32_t>(dma_len * 4),
                               .Stride = 0,
                               .NextConBlkAddr = 0};
    return {.txDMAConBlock = txConBlock, .rxDMAConBlock = rxConBlock};
}

DMASPI::DMASPI(std::array<DMASPISetting, 2> spiSettings, uintptr_t gpioMmapPtr, uintptr_t dmaMmapPtr,
               uintptr_t spiMmapPtr)
    : spiSettings_(spiSettings), gpioMmapPtr_(gpioMmapPtr), dmaMmapPtr_(dmaMmapPtr), spiMmapPtr_(spiMmapPtr)
{
    constexpr auto dmaLen = sizeof(CommandMessage) / 4;
    auto conBlockMemBlock = UncachedMemBlock_alloc(Page_Size);
    auto rxDataMemBlock = UncachedMemBlock_alloc(Page_Size);
    auto txDataMemBlock = UncachedMemBlock_alloc(Page_Size);
    auto conBlocks = span<DMAControlBlock, Page_Size / sizeof(DMAControlBlock)>(
        const_cast<DMAControlBlock *>(reinterpret_cast<volatile DMAControlBlock *>(conBlockMemBlock.mem.data())),
        Page_Size / sizeof(DMAControlBlock));
    auto rxBuffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(rxDataMemBlock.mem.data()), Page_Size / 4);
    auto txBuffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(txDataMemBlock.mem.data()), Page_Size / 4);

    conBlockMemBlock_ = conBlockMemBlock;
    conBlocks_ = conBlocks;
    txBuffer_ = txBuffer;
    rxBuffer_ = rxBuffer;

    auto spi0ConBlocks = SetupControlBlocks(0, dmaLen, UncachedMemBlock_to_physical(&txDataMemBlock, &txBuffer[0]),
                                            UncachedMemBlock_to_physical(&rxDataMemBlock, &rxBuffer[0]));
    auto spi6ConBlocks = SetupControlBlocks(6, dmaLen, UncachedMemBlock_to_physical(&txDataMemBlock, &txBuffer[256]),
                                            UncachedMemBlock_to_physical(&rxDataMemBlock, &rxBuffer[256]));
    conBlocks[0] = spi0ConBlocks.txDMAConBlock;
    conBlocks[1] = spi6ConBlocks.txDMAConBlock;
    conBlocks[10] = spi0ConBlocks.rxDMAConBlock;
    conBlocks[11] = spi6ConBlocks.rxDMAConBlock;
    InitialiseSPI(0, gpioMmapPtr, spiMmapPtr, spiSettings[0].SPI_CLK);
    InitialiseSPI(6, gpioMmapPtr, spiMmapPtr, spiSettings[1].SPI_CLK);
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
    int offset;
    if (spiNum == 0)
        offset = 0;
    else
        offset = 256;
    rxBufs_.at(spiNum) = rxBuf;
    for (int i = 0; i < txBuf.size() / 4; i++)
    {
        txBuffer_[i + offset] =
            (txBuf[i * 4] | (txBuf[i * 4 + 1] << 8) | (txBuf[i * 4 + 2] << 16) | (txBuf[i * 4 + 3] << 24));
    }
}
void DMASPI::Transceive()
{
    GPIO_Output<6> pin6(gpioMmapPtr_);
    pin6.Toggle();
    auto spiPtr1 = GetSPI<0>(spiMmapPtr_);
    auto spiPtr2 = GetSPI<6>(spiMmapPtr_);
    for (int i = 0; i < 2; i++)
    {
        auto txDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(0 + i).txDmaNum);
        auto rxDmaPtr = Get_DMA(dmaMmapPtr_, spiSettings_.at(0 + i).rxDmaNum);
        // Reset channel
        txDmaPtr->CtrlAndStatus |= 0b1 << 31;
        rxDmaPtr->CtrlAndStatus |= 0b1 << 31;
        txDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_, &conBlocks_[0 + i]);
        rxDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&conBlockMemBlock_, &conBlocks_[10 + i]);
        txDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
        rxDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    }
    std::this_thread::sleep_for(50us);
    while ((spiPtr1->CS & (0b1u << 16u)) == 0u)
    {
        std::this_thread::sleep_for(1us);
    }
    spiPtr1->CS = spiPtr1->CS & ~(0b1 << 7); // Clear Transfer Active bit
    while ((spiPtr2->CS & (0b1u << 16u)) == 0u)
    {
        std::this_thread::sleep_for(1us);
    }
    spiPtr2->CS = spiPtr2->CS & ~(0b1 << 7); // Clear Transfer Active bit
    pin6.Toggle();
    for (int i = 0; i < rxBufs_.at(0).size() / 4; i++)
    {
        uint32_t rxBytes = rxBuffer_[i];
        rxBufs_.at(0)[i*4] = rxBytes & 0b11111111;
        rxBufs_.at(0)[i*4+1] = rxBytes & (0b11111111 << 8);
        rxBufs_.at(0)[i*4+2] = rxBytes & (0b11111111 << 16);
        rxBufs_.at(0)[i*4+3] = rxBytes & (0b11111111 << 24);
    }
    for (int i = 0; i < rxBufs_.at(6).size() / 4; i++)
    {
        uint32_t rxBytes = rxBuffer_[i+256];
        rxBufs_.at(6)[i*4] = rxBytes & 0b11111111;
        rxBufs_.at(6)[i*4+1] = rxBytes & (0b11111111 << 8);
        rxBufs_.at(6)[i*4+2] = rxBytes & (0b11111111 << 16);
        rxBufs_.at(6)[i*4+3] = rxBytes & (0b11111111 << 24);
    }
    pin6.Toggle();
}

} // namespace robot_interface2
#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include "raspiHardware/UncachedMem.hpp"
#include <chrono>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;

void SetupControlBlocksTx(uint8_t spiNum, uint16_t dma_len, uintptr_t spiPhysAddr,
                          const UncachedMemBlock &conBlocksMemBlk,
                          span<volatile DMAControlBlock, Page_Size / sizeof(DMAControlBlock)> conBlocks,
                          const UncachedMemBlock &txDataMemBlk, span<volatile uint32_t, Page_Size / 4> txBuffer)
{
    uint8_t rxNum;
    uint8_t txNum;
    switch (spiNum)
    {
    case 0:
        rxNum = 7;
        txNum = 6;
        break;
    case 1:
        rxNum = 18;
        txNum = 16;
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
                                 .WaitCycle = 20};
    uint32_t tempTi;
    std::memcpy(&tempTi, &ti, sizeof(uint32_t));
    std::cout << "writing to con blocks1" << std::endl;
    conBlocks[0].TI = tempTi;
    conBlocks[0].SourceAddr = UncachedMemBlock_to_physical(&txDataMemBlk, &txBuffer[0]);
    conBlocks[0].DestAddr = spiPhysAddr + 0x04;
    conBlocks[0].TxLen = dma_len * 4;
    conBlocks[0].Stride = 0;
    conBlocks[0].NextConBlkAddr = 0;
}

void SetupControlBlocksRx(uint8_t spiNum, uint16_t dma_len, uintptr_t spiPhysAddr,
                          const UncachedMemBlock &conBlocksMemBlk,
                          span<volatile DMAControlBlock, Page_Size / sizeof(DMAControlBlock)> conBlocks,
                          const UncachedMemBlock &rxDataMemBlk, span<volatile uint32_t, Page_Size / 4> rxBuffer)
{
    uint8_t rxNum;
    uint8_t txNum;
    switch (spiNum)
    {
    case 0:
        rxNum = 7;
        txNum = 6;
        break;
    case 1:
        rxNum = 18;
        txNum = 16;
        break;
    case 6:
        rxNum = 27;
        txNum = 23;
        break;
    default:
        throw std::runtime_error("Wrong spi num for setting up DMA control blocks");
    }
    DMATransferInformation ti = DMATransferInformation{.DestAddrIncrement = true,
                                                       .DestTransferWidth = DMATransferWidth::Width32,
                                                       .DestWriteUseDREQ = false,
                                                       .SrcAddrIncrement = false,
                                                       .SrcTransferWidth = DMATransferWidth::Width32,
                                                       .SrcReadUseDREQ = true,
                                                       .PeriphMapNum = rxNum,
                                                       .WaitCycle = 1};
    uint32_t tempTi;
    std::memcpy(&tempTi, &ti, sizeof(uint32_t));

    std::cout << "writing to con blocks2" << std::endl;
    conBlocks[2].TI = tempTi;
    conBlocks[2].SourceAddr = spiPhysAddr + 0x04;
    conBlocks[2].DestAddr = UncachedMemBlock_to_physical(&rxDataMemBlk, &rxBuffer[0]);
    conBlocks[2].TxLen = dma_len * 4;
    conBlocks[2].Stride = 0;
    conBlocks[2].NextConBlkAddr = 0;
}

void SetupSPI(volatile SPIRegisters *spi)
{
    spi->CLK = 20;
    // Clear FIFO
    spi->CS = 0b11u << 4u;
    // Set DMAEN, set ADCS (automatically de assert CS, used by DMA), Use 32bit FIFO write
    spi->CS = 0b1 << 8 | 0b1 << 11 | 0b1 << 25;
    spi->DLEN = 108;
}

int main()
{

    uintptr_t gpioMmapPtr = mmapPeriph(GPIO_Base);
    uintptr_t spiMmapPtr = mmapPeriph(SPI_Base);
    uintptr_t dmaMmapPtr = mmapPeriph(DMA_Base);
    if (gpioMmapPtr == 0)
    {
        std::cerr << "Mmap failed" << std::endl;
        return -1;
    }
    uint8_t spiNum = 0;

    SetAF<8>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<9>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<10>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<11>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);

    auto spiPtr = GetSPI<0>(spiMmapPtr);
    auto spi_phys_addr = 0x7e204000 + spiNum * 0x200;

    auto control_block_mem_block = UncachedMemBlock_alloc(Page_Size);
    auto rx_data_mem_block = UncachedMemBlock_alloc(Page_Size);
    auto tx_data_mem_block = UncachedMemBlock_alloc(Page_Size);

    auto con_blocks = span<volatile DMAControlBlock, Page_Size / sizeof(DMAControlBlock)>(
        reinterpret_cast<volatile DMAControlBlock *>(control_block_mem_block.mem.data()), Page_Size / sizeof(DMAControlBlock));

    auto rx_buffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(rx_data_mem_block.mem.data()), 1024);
    auto tx_buffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(tx_data_mem_block.mem.data()), 1024);
    tx_buffer[0] = (static_cast<uint32_t>(108) << 16) | (0b1 << 7);
    for (int i = 0; i < 27; i++)
    {
        uint32_t val = static_cast<uint32_t>(4 * i + 10) | (static_cast<uint32_t>((4 * i) + 11) << 8) |
                       (static_cast<uint32_t>((4 * i) + 12) << 16) | (static_cast<uint32_t>((4 * i) + 13) << 24);
        tx_buffer[i + 1] = val;
    }
    //    tx_buffer[256] = tx_buffer[0];
    //    for (int i = 0; i < 27; i++)
    //    {
    //        tx_buffer[i + 1 + 256] = i + 10 + 256;
    //    }

    GPIO_Output<6> pin6(gpioMmapPtr);
    GPIO_Output<16> pin16(gpioMmapPtr);
    GPIO_Output<26> pin26(gpioMmapPtr);
    SetupControlBlocksTx(spiNum, 28, spi_phys_addr, control_block_mem_block, con_blocks, tx_data_mem_block, tx_buffer);
    SetupControlBlocksRx(spiNum, 27, spi_phys_addr, control_block_mem_block, con_blocks, rx_data_mem_block, rx_buffer);
    auto txDmaPtr = Get_DMA<8>(dmaMmapPtr);
    auto rxDmaPtr = Get_DMA<9>(dmaMmapPtr);
    // Reset channel
    txDmaPtr->CtrlAndStatus |= 0b1 << 31;
    rxDmaPtr->CtrlAndStatus |= 0b1 << 31;

    txDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &con_blocks[0]);
    rxDmaPtr->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &con_blocks[2]);
    SetupSPI(spiPtr);
    // AXI priority 9, Panic priority 9, Disable debug pause
    std::this_thread::sleep_for(100ms);
    pin6.Toggle();
    txDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    rxDmaPtr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    //    pin6.Toggle();
    //    std::this_thread::sleep_for(200ms);
    //    txDmaPtr->CtrlAndStatus |= 0b1;
    //    rxDmaPtr->CtrlAndStatus |= 0b1;
    //    pin6.Toggle();
    std::this_thread::sleep_for(50us);
    while ((spiPtr->CS & (0b1u << 16u)) == 0u)
    {
        std::this_thread::sleep_for(1us);
    }
    pin6.Toggle();
    std::this_thread::sleep_for(100ms);
    std::cout << "Rx data:" << std::endl;
    for (int i = 0; i < 27; i++)
    {
        std::cout << std::hex << rx_buffer[i] << ' ';
    }
    std::cout << std::endl;
}
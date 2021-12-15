#include "raspiHardware/DMASPI.hpp"
#include "raspiHardware/GPIO.hpp"
#include "raspiHardware/Mmap.hpp"
#include "raspiHardware/UncachedMem.hpp"
#include <chrono>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;

void SetupControlBlocks(uint8_t spiNum, uint16_t dma_len, uintptr_t txPhysAddr, uintptr_t rxPhysAddr,
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
                                 .WaitCycle = 20};
    uint32_t tempTi;
    std::memcpy(&tempTi, &ti, sizeof(uint32_t));
    std::cout << "writing to con blocks1" << std::endl;
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
                                      .WaitCycle = 10};
    uint32_t tempTi2;
    std::memcpy(&tempTi2, &ti2, sizeof(uint32_t));
    rxConBlock.TI = tempTi2;
    rxConBlock.SourceAddr = static_cast<uint32_t>(spiPhysAddr + 0x04),
    rxConBlock.DestAddr = static_cast<uint32_t>(rxPhysAddr);
    rxConBlock.TxLen = static_cast<uint32_t>(dma_len * 4);
    rxConBlock.Stride = 0;
    rxConBlock.NextConBlkAddr = 0;
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
    uint8_t spiNum1 = 0;
    uint8_t spiNum2 = 6;

    SetAF<8>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<9>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<10>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    SetAF<11>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF0);
    // Set AF3 for GPIO 18,19,20,21
    SetAF<18>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<19>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<20>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);
    SetAF<21>(reinterpret_cast<uintptr_t>(gpioMmapPtr), GPIO_AlternateFunc::AF3);

    auto spiPtr = GetSPI<0>(spiMmapPtr);
    auto spiPtr2 = GetSPI<6>(spiMmapPtr);

    auto control_block_mem_block = UncachedMemBlock_alloc(Page_Size);
    auto rx_data_mem_block = UncachedMemBlock_alloc(Page_Size);
    auto tx_data_mem_block = UncachedMemBlock_alloc(Page_Size);

    auto con_blocks = span<volatile DMAControlBlock, Page_Size / sizeof(DMAControlBlock)>(
        reinterpret_cast<volatile DMAControlBlock *>(control_block_mem_block.mem.data()),
        Page_Size / sizeof(DMAControlBlock));

    auto rx_buffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(rx_data_mem_block.mem.data()), 1024);
    auto tx_buffer = span<volatile uint32_t, Page_Size / 4>(
        reinterpret_cast<volatile uint32_t *>(tx_data_mem_block.mem.data()), 1024);
    tx_buffer[0] = (static_cast<uint32_t>(108) << 16) | (0b1 << 7);
    tx_buffer[256] = (static_cast<uint32_t>(108) << 16) | (0b1 << 7);
    for (int i = 0; i < 27; i++)
    {
        uint32_t val = static_cast<uint32_t>(4 * i + 10) | (static_cast<uint32_t>((4 * i) + 11) << 8) |
                       (static_cast<uint32_t>((4 * i) + 12) << 16) | (static_cast<uint32_t>((4 * i) + 13) << 24);
        uint32_t val2 = static_cast<uint32_t>(4 * i +128) | (static_cast<uint32_t>((4 * i) + 129) << 8) |
                       (static_cast<uint32_t>((4 * i) + 130) << 16) | (static_cast<uint32_t>((4 * i) + 131) << 24);
        tx_buffer[i + 1] = val;
        tx_buffer[i+1+256] = val2;
    }
    //    tx_buffer[256] = tx_buffer[0];
    //    for (int i = 0; i < 27; i++)
    //    {
    //        tx_buffer[i + 1 + 256] = i + 10 + 256;
    //    }

    GPIO_Output<6> pin6(gpioMmapPtr);
    GPIO_Output<16> pin16(gpioMmapPtr);
    GPIO_Output<26> pin26(gpioMmapPtr);
    auto txDmaPtr1 = Get_DMA<0>(dmaMmapPtr);
    auto rxDmaPtr1 = Get_DMA<1>(dmaMmapPtr);
    auto txDmaPtr2 = Get_DMA<2>(dmaMmapPtr);
    auto rxDmaPtr2 = Get_DMA<3>(dmaMmapPtr);
    // Reset channel
    txDmaPtr1->CtrlAndStatus |= 0b1 << 31;
    rxDmaPtr1->CtrlAndStatus |= 0b1 << 31;
    txDmaPtr2->CtrlAndStatus |= 0b1 << 31;
    rxDmaPtr2->CtrlAndStatus |= 0b1 << 31;
    {

        uintptr_t txPhysAddr = UncachedMemBlock_to_physical(&tx_data_mem_block, &tx_buffer[0]);
        uintptr_t rxPhysAddr = UncachedMemBlock_to_physical(&rx_data_mem_block, &rx_buffer[0]);
        auto &rxConBlock = con_blocks[10];
        auto &txConBlock = con_blocks[0];
        SetupControlBlocks(spiNum1, 27, txPhysAddr, rxPhysAddr, txConBlock, rxConBlock);

        txDmaPtr1->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &txConBlock);
        rxDmaPtr1->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &rxConBlock);
    }
    {

        uintptr_t txPhysAddr = UncachedMemBlock_to_physical(&tx_data_mem_block, &tx_buffer[256]);
        uintptr_t rxPhysAddr = UncachedMemBlock_to_physical(&rx_data_mem_block, &rx_buffer[256]);
        auto &rxConBlock = con_blocks[11];
        auto &txConBlock = con_blocks[1];
        SetupControlBlocks(spiNum2, 27, txPhysAddr, rxPhysAddr, txConBlock, rxConBlock);

        txDmaPtr2->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &txConBlock);
        rxDmaPtr2->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block, &rxConBlock);
    }

    SetupSPI(spiPtr);
    SetupSPI(spiPtr2);
    // AXI priority 9, Panic priority 9, Disable debug pause
    std::this_thread::sleep_for(100ms);
    pin6.Toggle();
    txDmaPtr1->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    rxDmaPtr1->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    txDmaPtr2->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
    rxDmaPtr2->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29 | 0b1;
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
    while ((spiPtr2->CS & (0b1u << 16u)) == 0u)
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
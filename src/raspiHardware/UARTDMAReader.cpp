#include "raspiHardware/UARTDMAReader.hpp"
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;
namespace robot_interface2
{

bool UARTDMAReader::printDebugInfo = false;

UARTDMAReader::UARTDMAReader(UARTRegisters *uart_ptr, DMARegisters *dma_ptr, uint8_t uart_num, uintptr_t gpioMmapPtr,
                             uint16_t dma_len, uint32_t baud)
    : uart_ptr_(uart_ptr), dma_ptr_(dma_ptr), baud_(baud), dma_len_(dma_len)
{
    if (dma_len > 256)
    {
        throw std::runtime_error("Dma length must be less than 256");
    }
    uart_phys_addr_ = 0x7e201000 + uart_num * 0x200;

    control_block_mem_block_ = UncachedMemBlock_alloc(Page_Size);
    data_mem_block_ = UncachedMemBlock_alloc(Page_Size);

    con_blocks_ = span<DMAControlBlock, Page_Size / sizeof(DMAControlBlock)>(
        reinterpret_cast<DMAControlBlock *>(control_block_mem_block_.mem.data()), 128);
    rx_buffer_ = span<uint32_t, Page_Size / 4>(reinterpret_cast<uint32_t *>(data_mem_block_.mem.data()), 1024);

    SetupControlBlocks(uart_num, dma_len);

    dma_ptr->CtrlAndStatus |= 0b1 << 31;
    std::this_thread::sleep_for(200ms);
    dma_ptr->CtrlBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block_, &con_blocks_[0]);
    // AXI priority 9, Panic priority 9, Disable debug pause
    dma_ptr->CtrlAndStatus = 9 << 16 | 9 << 20 | 0b1 << 29;
    std::this_thread::sleep_for(200ms);

    // Init UART GPIO
    InitUARTGPIO(gpioMmapPtr, uart_num);
}

void UARTDMAReader::InitUART(UARTRegisters *uart_ptr, uint32_t baud)
{
    // Disable UART
    uart_ptr->CR = 0;
    // Wait for the UART to reliably be stopped
    std::this_thread::sleep_for(150ms);

    // BAUDDIV = (FUARTCLK/(16 * Baud rate))
    // FUARTCLK of 45M (from empirical results), Baud of 2M, BAUDDIV = 1.40625, float portion is 0.40625*64 = 26
    constexpr uint32_t FUART_CLK = 45000000;
    uint32_t baud_int = FUART_CLK / 16 / baud;
    uint32_t baud_div = FUART_CLK / 16 * 64 / baud - baud_int * 64;
    uart_ptr->IBRD = baud_int;
    uart_ptr->FBRD = baud_div;

    // 8N1 mode, enable FIFO, disable line break
    uart_ptr->LCRH = 0;
    std::this_thread::sleep_for(100ms);
    uart_ptr->LCRH = 0b11 << 5 | 0b1 << 4;
    while (!(uart_ptr->FR & (0b1 << 4)))
    {
        std::cout << "===init rx not empty begin====" << std::endl;
        volatile uint32_t data = uart_ptr->DR;
        bool overrun_error = data & (0b1 << 11);
        bool break_error = data & (0b1 << 10);
        bool parity_error = data & (0b1 << 9);
        bool framing_error = data & (0b1 << 8);
        std::cout << "Got data " << static_cast<char>(data & 0b11111111) << " (" << (data & 0b11111111) << ") "
                  << ", Overrun: " << overrun_error << ", break: " << break_error << ", parity: " << parity_error
                  << ", framing: " << framing_error << std::endl;
        std::cout << "===init rx not empty end====" << std::endl;
    }

    uart_ptr->CR = 0b1 << 8 | 0b1 << 9; // Set TXE, Set RXE
    uart_ptr->DMACR = 0b1 << 1 | 0b1;   // set TXDMAE and RXDMAE
    // Wait for all configuration to be updated before starting UART
    std::this_thread::sleep_for(150ms);
    // Enable UART
    uart_ptr->CR |= 0b1;
}

void UARTDMAReader::SetupControlBlocks(uint8_t uart_num, uint16_t dma_len)
{
    uint8_t periph_num;
    switch (uart_num)
    {
    case 0:
        periph_num = 14;
        break;
    case 2:
        periph_num = 29;
        break;
    case 3:
        periph_num = 20;
        break;
    case 4:
        periph_num = 31;
        break;
    case 5:
        periph_num = 22;
        break;
    default:
        throw std::runtime_error("Wrong uart num for setting up DMA control blocks");
    }

    con_blocks_[0].TI = DMATransferInformation{.DestAddrIncrement = true,
                                               .DestTransferWidth = DMATransferWidth::Width32,
                                               .DestWriteUseDREQ = false,
                                               .SrcAddrIncrement = false,
                                               .SrcTransferWidth = DMATransferWidth::Width32,
                                               .SrcReadUseDREQ = true,
                                               .PeriphMapNum = periph_num,
                                               .WaitCycle = 0};
    con_blocks_[0].SourceAddr = uart_phys_addr_;
    con_blocks_[0].DestAddr = UncachedMemBlock_to_physical(&data_mem_block_, &rx_buffer_[0]);
    con_blocks_[0].TxLen = dma_len * 4;
    con_blocks_[0].Stride = 0;
    con_blocks_[0].NextConBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block_, &con_blocks_[1]);

    con_blocks_[1].TI = DMATransferInformation{.DestAddrIncrement = true,
                                               .DestTransferWidth = DMATransferWidth::Width32,
                                               .DestWriteUseDREQ = false,
                                               .SrcAddrIncrement = false,
                                               .SrcTransferWidth = DMATransferWidth::Width32,
                                               .SrcReadUseDREQ = true,
                                               .PeriphMapNum = periph_num,
                                               .WaitCycle = 0};
    con_blocks_[1].SourceAddr = uart_phys_addr_;
    con_blocks_[1].DestAddr = UncachedMemBlock_to_physical(&data_mem_block_, &rx_buffer_[256]);
    con_blocks_[1].TxLen = dma_len * 4;
    con_blocks_[1].Stride = 0;
    con_blocks_[1].NextConBlkAddr = UncachedMemBlock_to_physical(&control_block_mem_block_, &con_blocks_[0]);
}

UARTDMAReader::~UARTDMAReader()
{
    Stop();
    if (!control_block_mem_block_.mem.empty())
    {
        UncachedMemBlock_free(control_block_mem_block_);
    }
    if (!data_mem_block_.mem.empty())
    {
        UncachedMemBlock_free(data_mem_block_);
    }
    //        con_blocks_ = span<DMAControlBlock, Page_Size/sizeof(DMAControlBlock)>(nullptr, 128);
    //        rx_buffer_ = span<uint32_t, Page_Size/4>(nullptr, 1024);
}

bool WaitForDmaComplete(DMARegisters *dma_ptr, std::atomic<bool> &stop_requested, uint32_t &next_ctrl_blk_addr)
{
    // wait for NextCtrlBlk address to change
    while (dma_ptr->NextCtrlBlk == next_ctrl_blk_addr)
    {
        std::this_thread::sleep_for(1ms);
        if (stop_requested)
        {
            return false;
        }
    }
    return true;
}

std::array<uint8_t, 256> UARTDMAReader::ReadBufferData(span<uint32_t> buf)
{
    std::array<uint8_t, 256> data_byte_array{};
    for (int i = 0; i < buf.size(); i++)
    {
        auto data = buf[i];
        if (printDebugInfo)
        {
            auto error_flags = data & 0b111100000000;
            if (error_flags)
            {
                if (error_flags & (0b1 << 8))
                {
                    fprintf(stderr, "Framing error on byte with data 0x%x, index: %d\n", data & 0b11111111, i);
                }
                if (error_flags & (0b1 << 9))
                {
                    fprintf(stderr, "Parity error on byte with data 0x%x, index: %d\n", data & 0b11111111, i);
                }
                if (error_flags & (0b1 << 10))
                {
                    fprintf(stderr, "Break error on byte with data 0x%x, index: %d\n", data & 0b11111111, i);
                }
                if (error_flags & (0b1 << 11))
                {
                    fprintf(stderr, "Overrun error on byte with data 0x%x, index: %d\n", data & 0b11111111, i);
                }
            }
        }
        data_byte_array[i] = data & 0b11111111;
    }
    return data_byte_array;
}

void UARTDMAReader::Run()
{
    const auto physical_con_blk0_addr = UncachedMemBlock_to_physical(&control_block_mem_block_, &con_blocks_[0]);
    const auto physical_con_blk1_addr = UncachedMemBlock_to_physical(&control_block_mem_block_, &con_blocks_[1]);

    // Enable DMA, wait for outstanding writes
    dma_ptr_->CtrlAndStatus |= 0b1u | 0b1u << 28u;

    InitUART(uart_ptr_, baud_);

//    printf("Src address: %X\n", dma_ptr_->SourceAddr);
//    printf("Dest address: %X\n", dma_ptr_->DestAddr);
//    printf("NextCtrlBlk address: %X\n", dma_ptr_->NextCtrlBlk);

    while (!stop_requested_)
    {
        auto wait_start = steady_clock::now();
        // wait for NextCtrlBlk address to change
        uint32_t next_ctrl_blk_addr = dma_ptr_->NextCtrlBlk;
        auto dma_complete = WaitForDmaComplete(dma_ptr_, stop_requested_, next_ctrl_blk_addr);
        if (!dma_complete) // if not complete means stop requested
            break;
        auto wait_end = steady_clock::now();
        auto wait_time_us = duration_cast<microseconds>(wait_end - wait_start).count();

        dma_ptr_->CtrlAndStatus |= 0b1 << 1; // Clear END flag

        next_ctrl_blk_addr = dma_ptr_->NextCtrlBlk;
        if (printDebugInfo)
        {
            fprintf(stderr, "Time waited: %ld us\n", wait_time_us);
            fprintf(stderr, "New NextCtrlBlk address: %X\n", next_ctrl_blk_addr);
            fprintf(stderr, "Current Dest address: %X\n", dma_ptr_->DestAddr);
        }
        int index_offset;
        if (next_ctrl_blk_addr == physical_con_blk0_addr)
        {
            // This means it is currently writing to control block 1 (since next is 0)
            // So we read from control block 0
            index_offset = 0;
        }
        else if (next_ctrl_blk_addr == physical_con_blk1_addr)
        {
            // This means it is currently writing to control block 0 (since next is 1)
            // So we read from control block 1
            index_offset = 256;
        }
        else
        {
            fprintf(stderr, "NextCtrlBlk address unknown: %X\n", next_ctrl_blk_addr);
            fprintf(stderr, "CtrlBlk0 address: %lX\n", physical_con_blk0_addr);
            fprintf(stderr, "CtrlBlk1 address: %lX\n", physical_con_blk1_addr);
            break;
        }

        auto data_byte_array = ReadBufferData(rx_buffer_.subspan(index_offset, dma_len_));
        for (auto &callback : callbacks_)
        {
            callback(span<uint8_t>(data_byte_array.data(), dma_len_));
        }
    }

    // Disable RXE for uart
    uart_ptr_->CR = uart_ptr_->CR & (~(0b1 << 9));
    std::this_thread::sleep_for(200ms);
    dma_ptr_->CtrlAndStatus = (dma_ptr_->CtrlAndStatus & ~(0b1)); // clear DMAEN bit to pause DMA
    std::this_thread::sleep_for(1000ms);
    std::array<uint8_t, 256> data_byte_array{};
    uintptr_t dest_addr = dma_ptr_->DestAddr;
    int index_offset;
    uint32_t dest_offset;
    if (dest_addr >= con_blocks_[0].DestAddr && dest_addr < con_blocks_[0].DestAddr + 0x400)
    {
        // Check that destination address is part of first block for the incomplete DMA transfer
        index_offset = 0;
        dest_offset = dest_addr - con_blocks_[0].DestAddr;
    }
    else if (dest_addr >= con_blocks_[1].DestAddr && dest_addr < con_blocks_[1].DestAddr + 0x400)
    {
        // Check that destination address is part of second block for the incomplete DMA transfer
        index_offset = 256;
        dest_offset = dest_addr - con_blocks_[1].DestAddr;
    }
    else
    {
        index_offset = 0;
        dest_offset = 0;
        fprintf(stderr, "Dest addr out of range for both Data addresses at : %lX\n", dest_addr);
        fprintf(stderr, "DataBlock0 address: %X\n", con_blocks_[0].DestAddr);
        fprintf(stderr, "DataBlock1 address: %X\n", con_blocks_[1].DestAddr);
    }
    int tx_left = dma_ptr_->TxLen;
    uint32_t data_written = dest_offset / 4;
    if (printDebugInfo)
    {
        fprintf(stderr, "Dest addr : %lX\n", dest_addr);
        fprintf(stderr, "DataBlock0 address: %X\n", con_blocks_[0].DestAddr);
        fprintf(stderr, "DataBlock1 address: %X\n", con_blocks_[1].DestAddr);
        fprintf(stderr, "Data offset: %X\n", dest_offset);
        fprintf(stderr, "Num data written before offset: %d\n", data_written);
        fprintf(stderr, "Tx left: %d\n", 128 - (tx_left / 4));
    }
    data_byte_array = ReadBufferData(span<uint32_t>(rx_buffer_.subspan(index_offset, data_written)));

    for (auto &callback : callbacks_)
    {
        callback(span<uint8_t>(data_byte_array.data(), data_written));
    }

    // Stop UART
    uart_ptr_->CR = 0;
    dma_ptr_->CtrlAndStatus |= 0b1 << 31; // Reset DMA
}
void UARTDMAReader::Stop()
{
    stop_requested_ = true;
}
void UARTDMAReader::AddCallback(const UARTDMAReader::CallbackFn &callback)
{
    callbacks_.push_back(callback);
}
void UARTDMAReader::InitUARTGPIO(uintptr_t gpioMmapPtr, uint8_t uartNum)
{
    switch (uartNum)
    {
    case 0:
        // Set AF0 for GPIO14,15 (UART0)
        SetAF<14>(gpioMmapPtr, GPIO_AlternateFunc::AF0);
        SetAF<15>(gpioMmapPtr, GPIO_AlternateFunc::AF0);
        break;
    case 2:
        // Set AF4 for GPIO0, 1 (UART2)
        SetAF<0>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        SetAF<1>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        break;
    case 3:
        // Set AF4 for GPIO4,5 (UART3)
        SetAF<4>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        SetAF<5>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        break;
    case 5:
        // Set AF4 for GPIO12,13 (UART5)
        SetAF<12>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        SetAF<13>(gpioMmapPtr, GPIO_AlternateFunc::AF4);
        break;
    default:
        throw std::runtime_error("Unsupported UART number given to initialise");
    }
}
} // namespace RobotInterface

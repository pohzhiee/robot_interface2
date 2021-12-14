#ifndef ROBOT_INTERFACE2_DMA_HPP
#define ROBOT_INTERFACE2_DMA_HPP
#include "Common.hpp"

constexpr uint32_t DMA_Offset = 0x007000;
constexpr uint32_t DMA_Periph_Width = 0x100;
constexpr uint32_t DMA_Base = Peripheral_Base + DMA_Offset;

enum DMATransferWidth : bool
{
    Width128 = true,
    Width32 = false
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked-bitfield-compat"
struct DMATransferInformation
{
    bool InterruptEnable : 1;
    bool TwoDimMode : 1;
    bool : 1;
    bool WaitWriteResp : 1;
    bool DestAddrIncrement : 1;
    DMATransferWidth DestTransferWidth : 1;
    bool DestWriteUseDREQ : 1;
    bool DestIgnoreWrite : 1;
    bool SrcAddrIncrement : 1;
    DMATransferWidth SrcTransferWidth : 1;
    bool SrcReadUseDREQ : 1;
    bool SrcIgnoreRead : 1;
    uint8_t BurstTxLength : 4;
    uint8_t PeriphMapNum : 5;
    uint8_t WaitCycle : 5;
    bool NoWideBurstWrite : 1;
    bool : 5;
} __attribute__((packed));
static_assert(sizeof(DMATransferInformation) == sizeof(uint32_t));
#pragma GCC diagnostic pop

struct DMAControlBlock
{
    DMATransferInformation TI;
    uint32_t SourceAddr;
    uint32_t DestAddr;
    uint32_t TxLen;
    uint32_t Stride;
    uint32_t NextConBlkAddr;
    uint32_t Reserved1;
    uint32_t Reserved2;
};
static_assert(sizeof(DMAControlBlock) == 32);

struct DMARegisters
{
    volatile uint32_t CtrlAndStatus;
    volatile uint32_t CtrlBlkAddr;
    volatile uint32_t TransferInformation;
    volatile uint32_t SourceAddr;
    volatile uint32_t DestAddr;
    volatile uint32_t TxLen;
    volatile uint32_t Stride;
    volatile uint32_t NextCtrlBlk;
    volatile uint32_t Debug;
};

struct DMA_CS_Reg
{
    bool Active : 1;
    bool EndFlag : 1;
    bool InterruptStatus : 1;
    bool DREQState : 1;
    bool Paused : 1;
    bool DREQStopsDMA : 1;
    bool WaitingForOutstandingWrites : 1;
    bool : 1;
    bool Error : 1;
    uint8_t : 7;
    uint8_t AXIPriority : 4;
    uint8_t PanicPriority : 4;
    uint8_t : 4;
    bool WaitForOutstandingWrites : 1;
    bool DisableDebugPause : 1;
    bool AbortDMA : 1;
    bool Reset : 1;
} __attribute__((packed));
static_assert(sizeof(DMA_CS_Reg) == sizeof(uint32_t));

template <unsigned N> constexpr DMARegisters *Get_DMA(uintptr_t addr)
{
    static_assert(N >= 0);
    static_assert(N < 15);
    return reinterpret_cast<DMARegisters *>(addr + N * DMA_Periph_Width);
}

DMARegisters *Get_DMA(uintptr_t addr, uint8_t dmaNum)
{
    return reinterpret_cast<DMARegisters *>(addr + dmaNum * DMA_Periph_Width);
}

#endif // ROBOT_INTERFACE2_DMA_HPP

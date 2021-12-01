#ifndef ROBOT_INTERFACE2_UNCACHEDMEM_HPP
#define ROBOT_INTERFACE2_UNCACHEDMEM_HPP
extern "C"
{
#include "mailbox.h"
};
#include "Common.hpp"
#include <cassert>

#define MEM_FLAG_DIRECT (1 << 2)
#define MEM_FLAG_COHERENT (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)
// ---- Memory mappping defines
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

// A memory block that represents memory that is allocated in physical
// memory and locked there so that it is not swapped out.
// It is not backed by any L1 or L2 cache, so writing to it will directly
// modify the physical memory (and it is slower of course to do so).
// This is memory needed with DMA applications so that we can write through
// with the CPU and have the DMA controller 'see' the data.
// The UncachedMemBlock_{alloc,free,to_physical}
// functions are meant to operate on these.
struct UncachedMemBlock
{
    span<volatile uint8_t> mem; // User visible value: the memory to use.

    //-- Internal representation.
    uint32_t bus_addr;
    uint32_t mem_handle;
};

// Allocate a block of memory of the given size (which is rounded up to the next
// full page). The memory will be aligned on a page boundary and zeroed out.
struct UncachedMemBlock UncachedMemBlock_alloc(size_t size);

// Free block previously allocated with UncachedMemBlock_alloc()
void UncachedMemBlock_free(UncachedMemBlock &block);

// Given a pointer to memory that is in the allocated block, return the
// physical bus addresse needed by DMA operations.
uintptr_t UncachedMemBlock_to_physical(const struct UncachedMemBlock *blk, volatile void *p);
#endif // ROBOT_INTERFACE2_UNCACHEDMEM_HPP

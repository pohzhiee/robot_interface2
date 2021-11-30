#include "raspiHardware/UncachedMem.hpp"

static int mbox_fd = -1; // used internally by the UncachedMemBlock-functions.

struct UncachedMemBlock UncachedMemBlock_alloc(size_t size)
{
    if (mbox_fd < 0)
    {
        mbox_fd = mbox_open();
        assert(mbox_fd >= 0); // Uh, /dev/vcio not there ?
    }
    // Round up to next full page.
    size = size % Page_Size == 0 ? size : (size + Page_Size) & ~(Page_Size - 1);

    UncachedMemBlock result;
    result.mem_handle = mem_alloc(mbox_fd, size, Page_Size, MEM_FLAG_L1_NONALLOCATING);
    result.bus_addr = mem_lock(mbox_fd, result.mem_handle);
    auto mem_ptr = mapmem(BUS_TO_PHYS(result.bus_addr), size);
    result.mem = span<uint8_t>(reinterpret_cast<uint8_t *>(mem_ptr), size);
    fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n", (int)size, result.mem.data(), result.bus_addr,
            BUS_TO_PHYS(result.bus_addr));
    assert(result.bus_addr); // otherwise: couldn't allocate contiguous block.
    // Use 32-bit addressing rather than using memset or any other library functions due to the potential to use 64-bit
    // addressing on arm64 Using 64-bit addressing will cause a bus error
    auto *ptr1 = (uint32_t *)result.mem.data();
    for (int i = 0; i < 1024; i++)
    {
        ptr1[i] = 0;
    }

    return result;
}

void UncachedMemBlock_free(UncachedMemBlock &block)
{
    if (block.mem.data() == nullptr)
        return;
    assert(mbox_fd >= 0); // someone should've initialized that on allocate.
    unmapmem(block.mem.data(), block.mem.size());
    mem_unlock(mbox_fd, block.mem_handle);
    mem_free(mbox_fd, block.mem_handle);
    block.mem = span<uint8_t>();
}

uintptr_t UncachedMemBlock_to_physical(const struct UncachedMemBlock *blk, void *p)
{
    uint32_t offset = (uint8_t *)p - (uint8_t *)blk->mem.data();
    assert(offset < blk->mem.size()); // pointer not within our block.
    return blk->bus_addr + offset;
}
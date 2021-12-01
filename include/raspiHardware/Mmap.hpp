#ifndef ROBOT_INTERFACE2_MMAP_HPP
#define ROBOT_INTERFACE2_MMAP_HPP
#include "Common.hpp"

#include <cstdio>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
static uintptr_t mmapPeriph(uint32_t periph_addr)
{
    int mem_fd;
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        perror("can't open /dev/mem: ");
        fprintf(stderr, "You need to run this as root!\n");
        return 0;
    }

    auto result = mmap(NULL, // Any adddress in our space will do
                       Page_Size,
                       PROT_READ | PROT_WRITE, // Enable r/w on GPIO registers.
                       MAP_SHARED,
                       mem_fd,     // File to map, in this case system memory
                       periph_addr // bcm register address
    );
    close(mem_fd);

    if (result == MAP_FAILED)
    {
        fprintf(stderr, "mmap error %p\n", result);
        return 0;
    }

    return reinterpret_cast<uintptr_t>(result);
}
#endif // ROBOT_INTERFACE2_MMAP_HPP

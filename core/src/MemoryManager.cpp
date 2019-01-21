// Copyright (c) 2018 All Rights Reserved WestBot

#include <cerrno>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/MemoryManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.MemoryManager" )

    // The start address and length of the Lightweight bridge
    const int HPS_TO_FPGA_LW_BASE = 0xFF200000;
    const int HPS_TO_FPGA_LW_SPAN = 0x0020000;
}

MemoryManager::MemoryManager()
    : _lwBridgeMap( 0 )
    , _devMemFd( 0 )
{
    if( 0 != init() )
    {
        tFatal( LOG ) << "Failed to init MemoryManager module";
        return;
    }
}

MemoryManager::~MemoryManager()
{
    deInit();
}

// Public
void* MemoryManager::mapModuleToMemory( uint32_t moduleBaseAddress )
{
    // Set the moduleMap to the correct offset within the RAM
    // (moduleBaseAddress need to be in "hps_0.h")
    return ( void* )( ( char* )_lwBridgeMap + moduleBaseAddress );
}

// Private Methods
int MemoryManager::init()
{
    // Open up the /dev/mem device (aka, RAM)
    _devMemFd = open( "/dev/mem", O_RDWR | O_SYNC );
    if( _devMemFd < 0 )
    {
        tCritical( LOG ) << "Devmem open: failed";
        return -1;
    }

    // mmap() the entire address space of the Lightweight bridge
    // so we can access our custom module
    _lwBridgeMap = ( uint32_t* )mmap(
        NULL,
        HPS_TO_FPGA_LW_SPAN,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        _devMemFd,
        HPS_TO_FPGA_LW_BASE );

    if( MAP_FAILED == _lwBridgeMap )
    {
        tCritical( LOG ) << "Devmem mmap: failed";
        close( _devMemFd );
        return -1;
    }

    return 0;
}

void MemoryManager::deInit()
{
    // Unmap everything and close the /dev/mem file descriptor
    int result = munmap( _lwBridgeMap, HPS_TO_FPGA_LW_SPAN );
    if( result < 0 )
    {
        tCritical( LOG ) << "Devmem munmap: failed";
        close( _devMemFd );
        return;
    }

    close( _devMemFd );
}

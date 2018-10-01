// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_MEMORYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_MEMORYMANAGER_HPP_

#include <stdint.h>

namespace WestBot {
namespace RobotRock {

/*!
* \brief MemoryManager is the manager which open the memory via an FD and
*        mmap it.
*/
class MemoryManager
{
public:
    /*!
    * \brief Default constructor.
    */
    MemoryManager();

    /*!
    * \brief Desctuctor.
    */
    ~MemoryManager();

    /*!
    * \brief Helper to map a module in the memory.
    *
    * \param moduleBaseAddress
    *
    * \return Return a pointer in the memory where the module is mapped.
    */
    void* mapModuleToMemory( uint32_t moduleBaseAddress );

private:
    /*!
    * \brief Private: Init the module.
    *
    * \return Return an int based on the operation failure.
    */
    int init();

    /*!
    * \brief Private: Deinit the module in memory.
    */
    void deInit();

private:
    void* _lwBridgeMap;
    int _devMemFd;
};

}
}

#endif // WESTBOT_ROBOTROCK_MEMORYMANAGER_HPP_

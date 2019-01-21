// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_MEMORY_HPP_
#define WESTBOT_ROBOTROCK_MEMORY_HPP_

#include <stdint.h>

#include "Layer.hpp"
#include "MemoryManager.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief Helper to write/read into the memory.
 */
class Memory
{
public:
    /*!
     * \brief Construct a Memory helper.
     *
     * \param manager The memory manager which open the RAM page.
     * \param baseAddress The base address of the layer.
     */
    Memory( MemoryManager& manager, uint32_t baseAddress );

    /*!
     * \brief Destructor.
     */
    ~Memory();

    /*!
     * \brief Reset the memory object. Reset the internal memory pointer.
     */
    void reset();

    /*!
     * \brief Read a data in the memory.
     *
     * \param offset Offset of the register.
     * \param size Size of the register.
     *
     * \return Return the data read.
     */
    uint32_t read( uint32_t offset, uint32_t size ) const;

    /*!
     * \brief Write a data in the memory.
     *
     * \param offset Offset of the register.
     * \param size Size of the block.
     * \param data Data to be written.
     */
    void write( uint32_t offset, uint32_t size, uint32_t data );

private:
    MemoryManager _memoryManager;
    Layer_t* _layerMap;
};

}
}

#endif // WESTBOT_ROBOTROCK_MEMORY_HPP_

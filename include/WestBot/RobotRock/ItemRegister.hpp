// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ITEMREGISTER_HPP_
#define WESTBOT_ROBOTROCK_ITEMREGISTER_HPP_

#include <memory>

#include "Memory.hpp"

namespace WestBot {
namespace RobotRock {

/*!
* \brief This class represents a memory item. Wraps a register from any layer.
*/
class ItemRegister
{
public:
    using Ptr = std::shared_ptr< ItemRegister >;

    /*!
    * \brief Construct an ItemRegister.
    *
    * \param layer The parent layer in memory (1,2 or 3).
    * \param offset The offset in the register.
    * \param size The size of the register item.
    */
    ItemRegister( Memory& layer, int offset, int size );

    /*!
    * \brief Write a float value in the register.
    *
    * \param data The float value to be written.
    */
    void write( float data );
    /*!
    * \brief Write a unsigned value in the register.
    *
    * \param data The uint value to be written.
    */
    void write( uint32_t data );
    /*!
    * \brief Write a signed value in the register.
    *
    * \param data The int value to be written.
    */
    void write( int32_t data );

    /*!
    * \brief Read a template value in the register.
    *
    * \return Return the value in template format.
    */
    template< typename T >
    T read() const
    {
        uint32_t val = _layer.read( _offset, _size );
        T* tmp = ( T* )( & val );
        return *tmp;
    }

private:
    Memory _layer;
    int _offset;
    int _size;
};

}
}

#endif // WESTBOT_ROBOTROCK_ITEMREGISTER_HPP_

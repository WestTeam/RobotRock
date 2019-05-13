// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_OUTPUTHW_HPP_
#define WESTBOT_ROBOTROCK_OUTPUTHW_HPP_

#include "ItemRegister.hpp"
#include "Output.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Output class allow to create a proxy of a FPGA
 *        output. It use the layer registers mapping.
 */
class OutputHw : public Output
{
public:
    /*!
     * \brief Constructor of Output.
     * \param outputRegister A reference to the output register.
     * \param name Name of the output object.
     */
    OutputHw( const ItemRegister::Ptr& outputRegister, const QString& name );

    /*!
     * \brief Write digital value on IO.
     * \param val The value to be written.
     */
    void digitalWrite( DigitalValue val ) override;

    /*!
     * \brief Read digital value on IO
     *
     * \return The digital value
     */
    DigitalValue digitalRead() override;

private:
    ItemRegister::Ptr _outputRegister;
};

}
}

#endif // WESTBOT_ROBOTROCK_OUTPUTHW_HPP_

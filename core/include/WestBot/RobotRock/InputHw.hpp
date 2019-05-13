// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_INPUTHW_HPP_
#define WESTBOT_ROBOTROCK_INPUTHW_HPP_

#include "Input.hpp"
#include "ItemRegister.hpp"

class QTimer;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Input class allow to create a proxy of a FPGA
 *        input. It use the layer registers mapping.
 */
class InputHw : public Input
{
public:
    /*!
     * \brief Constructor of Input.
     * \param inputRegister A reference to the input register.
     * \param name Name of the input object.
     */
    InputHw( const ItemRegister::Ptr& inputRegister, const QString& name );

private:
    /*!
     * \brief Check the input state and update it's internal state.
     */
    void check() override;

private:
    ItemRegister::Ptr _inputRegister;
    QTimer* _eventTimer;
};

}
}

#endif // WESTBOT_ROBOTROCK_INPUTHW_HPP_

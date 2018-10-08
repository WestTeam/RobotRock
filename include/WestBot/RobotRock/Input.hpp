// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_INPUT_HPP_
#define WESTBOT_ROBOTROCK_INPUT_HPP_

#include <memory>

#include <QObject>
#include <QString>

#include "Common.hpp"
#include "ItemRegister.hpp"

class QTimer;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Input class allow to create a proxy of a FPGA
 *        input. It use the layer registers mapping.
 */
class Input : public QObject
{
    Q_OBJECT

public:
    using Ptr = std::shared_ptr< Input >;

    /*!
     * \brief Constructor of Input.
     * \param inputRegister A reference to the input register.
     * \param name Name of the input object.
     */
    Input( const ItemRegister::Ptr& inputRegister, const QString& name );

    /*!
     * \brief Destructor.
     */
    ~Input() override = default;

    /*!
     * \brief Return the name of the input object.
     * \return Return QString.
     */
    const QString& name() const;

    /*!
     * \brief Allow to check if the input is toogled on or off.
     * \return Return a value from the enum state.
     *
     */
    DigitalValue digitalRead();

    /*!
     * \brief Check the input state and update it's internal state.
     */
    void check();

signals:
    /*!
     * \brief Notify of a stateChanged off the input.
     */
    void stateChanged( const DigitalValue& value );

private:
    ItemRegister::Ptr _inputRegister;
    QString _name;
    DigitalValue _digitalValue;
    QTimer* _eventTimer;
};

}
}

#endif // WESTBOT_ROBOTROCK_INPUT_HPP_

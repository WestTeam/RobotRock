// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_INPUT_HPP_
#define WESTBOT_ROBOTROCK_INPUT_HPP_

#include <memory>

#include <QObject>
#include <QString>

#include "Common.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Input class is an abstract class and defines what is a standard
 *        input.
 */
class Input : public QObject
{
    Q_OBJECT

public:
    using Ptr = std::shared_ptr< Input >;

    Input( const QString& name );

    /*!
     * \brief Destructor.
     */
    ~Input() override = default;

    /*!
     * \brief Return the name of the input object.
     * \return Return QString.
     */
    virtual const QString& name() const;

    /*!
     * \brief Allow to check if the input is toogled on or off.
     * \return Return a value from the enum state.
     *
     */
    virtual DigitalValue digitalRead();

signals:
    /*!
     * \brief Notify of a stateChanged off the input.
     */
    void stateChanged( const DigitalValue& value );

private:
    /*!
     * \brief Check the input state and update it's internal state.
     */
    virtual void check();

protected:
    QString _name;
    DigitalValue _digitalValue;
};

}
}

#endif // WESTBOT_ROBOTROCK_INPUT_HPP_

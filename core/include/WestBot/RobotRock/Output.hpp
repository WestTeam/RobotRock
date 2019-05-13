// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_OUTPUT_HPP_
#define WESTBOT_ROBOTROCK_OUTPUT_HPP_

#include <memory>

#include <QString>

#include "Common.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Output class is an abstract class and defines what is a generic
 *        ouput.
 */
class Output
{
public:
    using Ptr = std::shared_ptr< Output >;

    /*!
     * \brief Constructor of Output.
     * \param name Name of the output object.
     */
    Output( const QString& name );
    virtual ~Output() = default;

    /*!
     * \brief Return the name of the output object.
     * \return Return QString.
     */
    virtual const QString& name() const;

    /*!
     * \brief Write digital value on IO.
     * \param val The value to be written.
     */
    virtual void digitalWrite( DigitalValue val ) = 0;

    /*!
     * \brief Read digital value on IO
     *
     * \return The digital value
     */
    virtual DigitalValue digitalRead() = 0;

protected:
    QString _name;
    DigitalValue _digitalValue;
};

}
}

#endif // WESTBOT_ROBOTROCK_OUTPUT_HPP_

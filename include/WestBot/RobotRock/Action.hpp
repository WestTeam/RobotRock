// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ACTION_HPP_
#define WESTBOT_ROBOTROCK_ACTION_HPP_

#include <memory>

#include <QObject>

class QTimer;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Action class is a base class of all system actions.
 */
class Action : public QObject
{
    Q_OBJECT

public:
    using Ptr = std::shared_ptr< Action >;

    /*!
     * \brief List of available state of an action.
     */
    enum class State
    {
        Pending = 0,
        Running,
        Finished,
        InError,
        Flushed
    };

    /*!
     * \brief Constructor of Action.
     * \param parent Parent class for lifetime management.
     */
    Action( QObject* parent = nullptr );

    /*!
    * \brief Destructor
    */
    ~Action() override = default;

    /*!
    * \brief Virtual method to be override by daughter class.
    *        The base method allows to execute the action. This will push back
    *        the action on layer 1, 2 or 3 based on the action type.
    */
    virtual void execute() = 0;

    /*!
    * \brief Get the current state of the action.
    *
    * \return Return an enum state element.
    */
    State state() const;

    /*!
    * \brief Set the state of the action.
    *
    * \param state The new state of the action.
    */

    void setState( State state );

    /*!
    * \brief Check if the action is in error or not.
    *
    * \return If true, the action is in error, else not.
    */
    bool hasError() const;

signals:
    /*!
    * \brief Notify of a state changed.
    */
    void stateChanged();

    /*!
    * \brief Notify of the action completion.
    */
    void complete();

    /*!
    * \brief Notify that the action was skipped.
    */
    void skipped();

private:
    State _state;
    QTimer* _timeout;
};

}
}

#endif // WESTBOT_ROBOTROCK_ACTION_HPP_

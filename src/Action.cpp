// Copyright (c) 2018 All Rights Reserved WestBot

#include <QTimer>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Action.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Action" )
}

Action::Action( const QString& name, QObject* parent )
    : QObject( parent )
    , _state( Action::State::Pending )
    , _name( name )
{
    _timeout = new QTimer( this );

    connect(
        _timeout,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            tWarning( LOG ) << "Action timed out";
            _state = Action::State::TimedOut;
        } );
}

Action::State Action::state() const
{
    return _state;
}

void Action::setState( Action::State state )
{
    if( _state != state )
    {
        _state = state;
        emit stateChanged();
    }
}

bool Action::hasError() const
{
    return _state == Action::State::InError ||
           _state == Action::State::TimedOut;
}

const QString& Action::name() const
{
    return _name;
}

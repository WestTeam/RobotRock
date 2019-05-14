// Copyright (c) 2019 All Rights Reserved WestBot

#include <QTimer>

#include <WestBot/RobotRock/InputSimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

InputSimu::InputSimu( char key, const QString& name )
    : Input( name )
    , _key( key )
{
    _eventTimer = new QTimer( this );

    connect(
        _eventTimer,
        & QTimer::timeout,
        this,
        & InputSimu::check );

    _eventTimer->start( 100 );

    check();
}

void InputSimu::check()
{
    // TODO
}

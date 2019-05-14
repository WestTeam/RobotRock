// Copyright (c) 2019 All Rights Reserved WestBot

#include <QTimer>

#include <WestBot/RobotRock/InputHw.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

InputHw::InputHw( const ItemRegister::Ptr& inputRegister, const QString& name )
    : Input( name )
    , _inputRegister( inputRegister )
{
    _eventTimer = new QTimer( this );

    connect(
        _eventTimer,
        & QTimer::timeout,
        this,
        & InputHw::check );

    _eventTimer->start( 100 );

    check();
}

void InputHw::check()
{
    DigitalValue tmpVal;

    if( _inputRegister->read< uint32_t >() == 0x01 )
    {
       tmpVal = DigitalValue::ON;
    }
    else
    {
        tmpVal = DigitalValue::OFF;
    }

    if( tmpVal != _digitalValue )
    {
        _digitalValue = tmpVal;
        emit stateChanged( _digitalValue );
    }
}

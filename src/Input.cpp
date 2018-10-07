// Copyright (c) 2018 All Rights Reserved WestBot

#include <QTimer>

#include <WestBot/RobotRock/Input.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

Input::Input( const ItemRegister::Ptr& inputRegister, const QString& name )
    : _inputRegister( inputRegister )
    , _name( name )
    , _digitalValue( DigitalValue::OFF )
{
    _eventTimer = new QTimer( this );

    connect(
        _eventTimer,
        & QTimer::timeout,
        this,
        & Input::check );

    _eventTimer->start( 100 );

    check();
}

const QString& Input::name() const
{
    return _name;
}

DigitalValue Input::digitalRead()
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
	//check();
	//return _digitalValue;
	return tmpVal;
}

void Input::check()
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

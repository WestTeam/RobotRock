// Copyright (c) 2018 All Rights Reserved WestBot

#include <WestBot/RobotRock/Output.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

Output::Output( const ItemRegister::Ptr& outputRegister, const QString& name )
    : _outputRegister( outputRegister )
    , _name( name )
    , _digitalValue( DigitalValue::OFF )
{
    digitalRead();
}

const QString& Output::name() const
{
    return _name;
}

void Output::digitalWrite( DigitalValue val )
{
    if( val == DigitalValue::ON )
    {
        _outputRegister->write( 0x01 );
    }
    else
    {
        _outputRegister->write( 0x00 );
    }
}

DigitalValue Output::digitalRead()
{
    if( _outputRegister->read< uint32_t >() == 0x01 )
    {
        _digitalValue = DigitalValue::ON;
    }
    else
    {
        _digitalValue = DigitalValue::OFF;
    }

    return _digitalValue;
}

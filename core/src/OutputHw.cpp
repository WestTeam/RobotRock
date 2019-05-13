// Copyright (c) 2019 All Rights Reserved WestBot

#include <WestBot/RobotRock/OutputHw.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

OutputHw::OutputHw(
    const ItemRegister::Ptr& outputRegister,
    const QString& name )
    : Output( name )
    , _outputRegister( outputRegister )
{
    digitalRead();
}

void OutputHw::digitalWrite( DigitalValue val )
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

DigitalValue OutputHw::digitalRead()
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


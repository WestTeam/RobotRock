// Copyright (c) 2019 All Rights Reserved WestBot

#include <WestBot/RobotRock/OutputSimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

OutputSimu::OutputSimu( char key, const QString& name )
    : Output( name )
    , _key( key )
{
    digitalRead();
}

void OutputSimu::digitalWrite( DigitalValue val )
{
    if( val == DigitalValue::ON )
    {
        // TODO
    }
    else
    {
         // TODO
    }
}

DigitalValue OutputSimu::digitalRead()
{
    // TODO
    return _digitalValue;
}

// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <WestBot/RobotRock/Output.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

Output::Output( const QString& name )
    : _name( name )
    , _digitalValue( DigitalValue::OFF )
{
}

const QString& Output::name() const
{
    return _name;
}

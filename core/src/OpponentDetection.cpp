// Copyright (c) 2018 All Rights Reserved WestBot

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/OpponentDetection.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.OpponentDetection" )
}

OpponentDetection::OpponentDetection()
{
}

//
// Public methods
//
void OpponentDetection::methodFoo()
{
    // DO WHAT YOU WANT
    emit opponentDetected( 10, 20 );
}

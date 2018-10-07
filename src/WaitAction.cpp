// Copyright (c) 2018 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/WaitAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.WaitAction" )
}

WaitAction::WaitAction( int waitMs )
    : Action()
    , _waitMs( waitMs )
{
}

void WaitAction::execute()
{
    tDebug( LOG ) << "Running WaitAction";
    QThread::msleep( _waitMs );

    emit complete();
}

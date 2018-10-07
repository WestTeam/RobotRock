// Copyright (c) 2018 All Rights Reserved WestBot

#include <QCoreApplication>
#include <QThread>

#include <Defines.hpp>

#include <WestBot/RobotRock/GameThread.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.GameThread" )
}

GameThread::GameThread(
    const StrategyManager& strategyManager )
    : _strategyManager( strategyManager )
    , _color( Color::Unknown )
{
}

void GameThread::setGameColor(
    const Color& color )
{
    _color = color;
}

void GameThread::run()
{
    _strategyManager.doStrat( _color );
}

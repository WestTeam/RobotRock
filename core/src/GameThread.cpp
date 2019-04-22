// Copyright (c) 2018 All Rights Reserved WestBot

#include <WestBot/RobotRock/GameThread.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

GameThread::GameThread(
    const StrategyManager::Ptr& strategyManager,
    const Color& color )
    : _strategyManager( strategyManager )
    , _color( color )
{
}

const Color& GameThread::color() const
{
    return _color;
}

void GameThread::run()
{
    _strategyManager->doStrat( _color );
}

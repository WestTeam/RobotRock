// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_GAMETHREAD_HPP_
#define WESTBOT_ROBOTROCK_GAMETHREAD_HPP_

#include <QThread>
#include <QObject>

#include "Common.hpp"
#include "StrategyManager.hpp"

namespace WestBot {
namespace RobotRock {

class GameThread : public QThread
{
	Q_OBJECT

	void run();

public:
    GameThread( const StrategyManager& strategyManager );

    void setGameColor( const Color& color );

private:
    StrategyManager _strategyManager;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_GAMETHREAD_HPP_

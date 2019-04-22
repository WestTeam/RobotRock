// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_GAMETHREAD_HPP_
#define WESTBOT_ROBOTROCK_GAMETHREAD_HPP_

#include <memory>

#include <QThread>

#include "Common.hpp"
#include "StrategyManager.hpp"

namespace WestBot {
namespace RobotRock {

class GameThread : public QThread
{
	void run();

public:
    using Ptr = std::unique_ptr< GameThread >;

    GameThread(
        const StrategyManager::Ptr& strategyManager,
        const Color& color );

    const Color& color() const;

private:
    StrategyManager::Ptr _strategyManager;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_GAMETHREAD_HPP_

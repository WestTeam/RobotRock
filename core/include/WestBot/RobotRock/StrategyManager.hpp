// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

#include <memory>

#include "Common.hpp"
#include "TrajectoryManager.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief This class is an abstract class and defines what is a strategy manager
 */
class StrategyManager
{
public:
    using Ptr = std::shared_ptr< StrategyManager >;

    virtual ~StrategyManager() = default;

    virtual bool init( const TrajectoryManager::Ptr& trajectoryManager ) = 0;
    virtual void stop() = 0;

    virtual void doStrat( const Color& color ) = 0;

    virtual void buildStrat( const Color& color ) = 0;

    virtual void hardStop() = 0;

    virtual void obstacleToClose( bool avoid ) = 0;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

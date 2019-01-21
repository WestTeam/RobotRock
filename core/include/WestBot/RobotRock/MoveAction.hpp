// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_MOVEACTION_HPP_
#define WESTBOT_ROBOTROCK_MOVEACTION_HPP_

#include <memory>

#include "Action.hpp"
#include "TrajectoryManager.hpp"

namespace WestBot {
namespace RobotRock {

class MoveAction : public Action
{
public:
    using Ptr = std::shared_ptr< MoveAction >;

    MoveAction(
        TrajectoryManager& trajectoryManager,
        TrajectoryManager::TrajectoryType type,
        float theta,
        float distance,
        float x,
        float y,
        bool correction,
        bool doNotBlock = false );

    void execute() override;

private:
    TrajectoryManager& _trajectoryManager;
    TrajectoryManager::TrajectoryType _type;
    float _theta;
    float _distance;
    float _x;
    float _y;
    bool _correction;
    bool _doNotBlock;
};

}
}

#endif // WESTBOT_ROBOTROCK_MOVEACTION_HPP_

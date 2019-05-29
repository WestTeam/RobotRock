// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARMSMANAGERACTION_HPP_
#define WESTBOT_ROBOTROCK_ARMSMANAGERACTION_HPP_

#include <memory>

#include "Action.hpp"
#include "ArmsManager.hpp"

namespace WestBot {
namespace RobotRock {

class ArmsManagerAction : public Action
{
public:
    using Ptr = std::shared_ptr< ArmsManagerAction >;

    enum Type
    {
        SAFETY_CHECK = 0,
        INIT_POSITION, // (startup safe position)
        GET_PUCKS, // Grab without Storage (only left1 & left2 possible)
        GET_PUCKS_AND_STORE, // Grab and Store
        RELEASE_ALL_PUCKS_ACCELERATOR, // release one by one all pucks into accelerator
        RELEASE_ALL_PUCKS_SCALE, // release one by one all pucks into scale
        RELEASE_ALL_PUCKS_GROUND // release one by one all pucks on the ground
    };

    ArmsManagerAction(
        const ArmsManager::Ptr& armsManager,
        Type type,
        PuckPos *left1,
        PuckPos *left2,
        PuckPos *right1,
        PuckPos *right2,
        bool invArms//if true, left is assigned to right
        );

    void execute() override;

private:
    ArmsManager::Ptr _armsManager;
    Type _type;
    PuckPos* _left1;
    PuckPos* _left2;
    PuckPos* _right1;
    PuckPos* _right2;
};

}
}

#endif // WESTBOT_ROBOTROCK_ARMSMANAGERACTION_HPP_


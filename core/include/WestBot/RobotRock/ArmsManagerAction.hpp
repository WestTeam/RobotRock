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
        GET_PUCKS_ON_DISTRI_STEP1,
        GET_PUCKS_ON_DISTRI_STEP2,
        GET_PUCKS_ON_DISTRI_STEP3,
        GET_PUCKS_ON_DISTRI_ON_SIDE_STEP1, // prepare (arm not used safe mode, arm used replié mais pret)
        GET_PUCKS_ON_DISTRI_ON_SIDE_STEP2, // go get (we activate pump and move only UPPER ARM)
        GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3, // go back, and store if dist < 20.0
        RELEASE_PUCK_ACCELERATOR_STEP1, //
        RELEASE_PUCK_ACCELERATOR_STEP2, //
        RELEASE_PUCK_ACCELERATOR_STEP3, //
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
    bool _invArms;
    PuckPos* _left1;
    PuckPos* _left2;
    PuckPos* _right1;
    PuckPos* _right2;
};

}
}

#endif // WESTBOT_ROBOTROCK_ARMSMANAGERACTION_HPP_


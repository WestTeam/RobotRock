// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmsManagerAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmManagerAction" )
}

ArmsManagerAction::ArmsManagerAction(
    const ArmsManager::Ptr& armsManager,
    Type type,
    PuckPos *left1,
    PuckPos *left2,
    PuckPos *right1,
    PuckPos *right2,
    bool invArms//if true, left is assigned to right
        )
    : Action( "ArmsManagerAction" )
    , _armsManager( armsManager )
    , _type( type )
    , _left1( left1 )
    , _left2( left2 )
    , _right1( right1 )
    , _right2( right2 )
{
    if (invArms)
    {
        _left1 = right1;
        _left2 = right2;
        _right1 = left1;
        _right2 = left2;
    }
}


void ArmsManagerAction::execute()
{
    tDebug( LOG ) << "Running" << name() << "action";

    switch( _type )
    {
    case ArmsManagerAction::Type::SAFETY_CHECK:
        break;

    case ArmsManagerAction::Type::INIT_POSITION:
        break;

    case ArmsManagerAction::Type::GET_PUCKS:
        _armsManager->getPucks(_left1,_right1);
        break;

    case ArmsManagerAction::Type::GET_PUCKS_AND_STORE:
        _armsManager->getPucksAndStore(_left1,_left2,_right1,_right2);
        break;

    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_ACCELERATOR:
        _armsManager->releasePucksAcceletator();
        break;

    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_SCALE:
        _armsManager->releasePucksScale();
        break;

    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_GROUND:
        _armsManager->releasePucksGround();
        break;

    default:
        tWarning( LOG ) << "Not a valid ArmsManager action type:" << _type;
    }

    emit complete();
}

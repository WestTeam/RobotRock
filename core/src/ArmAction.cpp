// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmAction" )
}

ArmAction::ArmAction(
    const ArmHighLevel::Ptr& arm,
    ArmAction::Type type,
    double xMm,
    double yMm,
    double zMm )
    : Action( "ArmAction" )
    , _arm( arm )
    , _type( type )
    , _xMm( xMm )
    , _yMm( yMm )
    , _zMm( zMm )
{
}

void ArmAction::execute()
{
    tDebug( LOG ) << "Running" << name() << "action";

    switch( _type )
    {
    case ArmAction::Type::SAFETY_CHECK:
        break;

    case GROUND_PUCK_COLLECT:
        break;

    case DISTRIBUTOR_PUCK_COLLECT:
        break;

    case GOLD_DOOR_OPEN_CHECK:
        break;

    case GOLD_PUCK_COLLECT:
        break;

    case PUCK_STORE:
        break;

    case PUCK_UNSTORE:
        break;

    case PUCK_RELEASE:
        break;

    default:
        tWarning( LOG ) << "Not a valid arm action type:" << _type;
    }

    emit complete();
}

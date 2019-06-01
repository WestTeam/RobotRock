// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/RecalageAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.RecalageAction" )
}

RecalageAction::RecalageAction(
    const Recalage::Ptr& recalage,
    bool triggerOnce,
    bool continuous
        )
    : Action( "RecalageAction" )
    , _recalage( recalage )
    , _triggerOnce (triggerOnce)
    , _continuous (continuous)
{

}


void RecalageAction::execute()
{

    tDebug( LOG ) << "Running" << name() << "action";


    if (_continuous)
        _recalage->setCalibrationMode(_continuous);
    if (_triggerOnce)
        _recalage->triggerCalibration(true);

    emit complete();
}

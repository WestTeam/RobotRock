// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

//#define DEBUG
//#define SIMU
//#define USE_SIMULATOR

#include <signal.h>

#ifdef USE_SIMULATOR
#include <WestBot/RobotRock/SystemManagerSimu.hpp>
#else
#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/SystemManagerHw.hpp>
#endif

#include "StrategyManagerHomologation.hpp"
#include "StrategyManagerV1.hpp"
#include "StrategyManagerMatch1.hpp"
#include "StrategyManagerMatch2.hpp"
#include "StrategyManagerMatch3.hpp"
#include "StrategyManagerMatch4.hpp"
#include "StrategyManagerMatch5.hpp"

#include "StrategyManagerDebugPosition.hpp"


using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Main" )
}

int main( int argc, char *argv[] )
{
    QCoreApplication app( argc, argv );

    signal (SIGINT,  [](int) {QCoreApplication::quit();});
    signal (SIGKILL,  [](int) {QCoreApplication::quit();});

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif



    //StrategyManagerV1::Ptr strategyV1 =
    //    std::make_shared< StrategyManagerV1 >();

    StrategyManagerHomologation::Ptr strategyManagerMatch5 =
        std::make_shared< StrategyManagerMatch5 >();


#ifndef USE_SIMULATOR

    Hal::Ptr hal = std::make_shared< Hal >();
    SystemManagerHw system( hal, strategyManagerMatch5 );


    tInfo( LOG ) << "==== System started ! ==== ";

    if( ! system.init() )
    {
        tFatal( LOG ) << "Failed to init system manager";
    }

  #ifdef DEBUG
    hal->dump();
  #endif

  #ifdef SIMU
    hal->_modeSimu.write( 1 );
  #endif
#else

    SystemManagerSimu system( strategyManagerMatch5 );

    tInfo( LOG ) << "==== System started ! ==== ";

    if( ! system.init() )
    {
        tFatal( LOG ) << "Failed to init system manager";
    }

#endif

    tInfo( LOG ) << "==== System ready ! ==== ";

    return app.exec();
}

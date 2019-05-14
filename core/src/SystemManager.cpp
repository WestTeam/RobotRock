// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SystemManager" )
}

SystemManager::SystemManager( QObject* parent )
    : QObject( parent )
    , _startButton( nullptr )
    , _colorButton( nullptr )
    , _hardstopButton( nullptr )
    , _ledYellow( nullptr )
    , _ledBlue( nullptr )
    , _color( Color::Unknown )
{
    connect(
        & _gameTimer,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            tInfo( LOG ) << "Game ended";
            stop();
        } );

    // Task to notify that the robot is alive
    connect(
        & _aliveTimer,
        & QTimer::timeout,
        this,
        & SystemManager::robotAlive );
}

//
// Private methods
//
void SystemManager::blinkColorLed()
{
    _aliveTimer.start( 250 );
}

void SystemManager::robotAlive()
{
    if( _color == Color::Blue )
    {
        _ledBlue->digitalWrite( DigitalValue::OFF );
        QThread::msleep( 250 );
        _ledBlue->digitalWrite( DigitalValue::ON );
        QThread::msleep( 250 );
    }
    else
    {
        _ledYellow->digitalWrite( DigitalValue::OFF );
        QThread::msleep( 250 );
        _ledYellow->digitalWrite( DigitalValue::ON );
        QThread::msleep( 250 );
    }
}

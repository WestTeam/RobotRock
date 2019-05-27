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

class Alive : public QThread
{
public:
    Alive(
        const Output::Ptr& ledYellow,
        const Output::Ptr& ledBlue )
        : _isRunning( false )
        , _color( Color::Unknown )
        , _ledYellow( ledYellow )
        , _ledBlue( ledBlue )
    {
    }

    ~Alive() override = default;

    void init( Color color )
    {
        _color = color;
        start();
    }

    void stop()
    {
        _isRunning = false;
    }

private:
    void run() override
    {
        while( _isRunning )
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
    }

private:
    bool _isRunning;
    Color _color;
    const Output::Ptr& _ledYellow;
    const Output::Ptr& _ledBlue;
};

static Alive* _alive = nullptr;

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
}

void SystemManager::stop()
{
    if (_alive)
        _alive->stop();
}

//
// Private methods
//
void SystemManager::blinkColorLed()
{
    _alive = new Alive( _ledYellow, _ledBlue );
    _alive->init( _color );
}

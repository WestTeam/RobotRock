// Copyright (c) 2018 All Rights Reserved WestBot

#include <QThread>

#include <Macros.hpp>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SystemManager" )

    const int GAME_DURATION = 90 * 1000; // 90s
}

SystemManager::SystemManager( Hal& hal, QObject* parent )
    : QObject( parent )
    , _startButton(
          new Input(
              std::make_shared< ItemRegister >( _hal._input0 ),
             "Tirette" ) )
    , _colorButton(
          new Input(
              std::make_shared< ItemRegister >( _hal._input1 ),
              "Color" ) )
    , _hardstopButton(
          new Input(
              std::make_shared< ItemRegister >( _hal._input2 ),
              "AU" ) )
    , _ledYellow(
          new Output(
              std::make_shared< ItemRegister >( _hal._output0 ),
              "yellow" ) )
    , _ledBlue(
          new Output(
              std::make_shared< ItemRegister >( _hal._output2 ),
              "blue" ) )
    , _color( Color::Unknown )
    , _trajectoryManager( _hal, _recalage )
    , _systemMode( SystemManager::SystemMode::Full )
{
    connect(
        & _gameTimer,
        & QTimer::timeout,
       this,
       [ this ]()
       {
           tInfo( LOG ) << "Game ended";
           _aliveTimer.stop();
       } );


    // Task to notify that the robot is alive
    connect(
        & _aliveTimer,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            tDebug( LOG ) << "Robot alive";
            robotAlive();
        } );

    connect(
        _startButton.get(),
        & Input::stateChanged,
        this,
        [ this ]( const DigitalValue& value )
        {
            tInfo( LOG ) << "Start button changed to:" << value;

            if( value == DigitalValue::ON &&
                _hardstopButton->digitalRead() == DigitalValue::OFF )
            {
                start();
            }
        } );

    connect(
        _colorButton.get(),
        & Input::stateChanged,
        this,
        [ this ]( const DigitalValue& value )
        {
            tInfo( LOG ) << "Color button changed to:" << value;
            displayColor( value );
        } );

    connect(
        _hardstopButton.get(),
        & Input::stateChanged,
        this,
        [ this ]( const DigitalValue& value )
        {
            tInfo( LOG ) << "Hardstop button changed to:" << value;
            _gameTimer.stop();
            _aliveTimer.stop();
        } );
}

SystemManager::~SystemManager()
{
    stop();
}

//
// Public methods
//

bool SystemManager::init()
{
    // Always reset the system at startup
    reset();

    // Config PID Distance
    _hal._pidDistanceEnable.write( 0 );
    _hal._pidDistanceOverride.write( 0 );
    _hal._pidDistanceInverted.write( 0 );
    _hal._pidDistanceKp.write( ( float ) 2000.0 );
    _hal._pidDistanceKi.write( ( float ) 0.0 );
    _hal._pidDistanceKd.write( ( float ) 0.0 );

    _hal._pidDistanceSpeed.write( ( float ) 0.01 );
    _hal._pidDistanceAcceleration.write( ( float ) 0.0001 );
    _hal._pidDistanceSaturation.write( 25000 );

    _hal._pidDistanceTarget.write( _hal._pidDistancePosition.read< float >() );
    _hal._pidDistanceEnable.write( 1 );

    // Config PID Angle
    _hal._pidAngleEnable.write( 0 );
    _hal._pidAngleOverride.write( 0 );
    _hal._pidAngleInverted.write( 1 );
    _hal._pidAngleKp.write( ( float ) 500000.0 );
    _hal._pidAngleKi.write( ( float ) 0.0 );
    _hal._pidAngleKd.write( ( float ) 0.0 );

    _hal._pidAngleSpeed.write( ( float ) 0.0001 );
    _hal._pidAngleAcceleration.write( ( float ) 0.00000002 );
    _hal._pidAngleSaturation.write( 25000 );

    _hal._pidAngleTarget.write( _hal._pidAnglePosition.read< float >() );
    _hal._pidAngleEnable.write( 1 );

    if( ! _recalage.init( _hal ) )
    {
        tWarning( LOG ) << "Failed to init recalage module";
        return false;
    }

    _trajectoryManager.init();

    // Override output registers
    _hal._outputOverride.write( 0x01010101 );

    displayColor( _colorButton->digitalRead() );

    tDebug( LOG ) << "Start button is:" << _startButton->digitalRead();
    tDebug( LOG ) << "Color button is:" << _colorButton->digitalRead();

    return true;
}

void SystemManager::start()
{
    tInfo( LOG ) << "System starting...";

    initRecalage();

    displayColor( _colorButton->digitalRead() );

    tInfo( LOG ) << "System started with color " << _color;

    blinkColorLed();

    _gameTimer.start( GAME_DURATION );
    _gameTimer.setSingleShot( true );

   // TODO: DO STRAT
}

void SystemManager::stop()
{
    tInfo( LOG ) << "System stopped";
}

void SystemManager::reset()
{
    _hal._colorEnable.write( 0 );

    _hal._resetAll.write( 1 );

    _hal.clearRegisters();

    _hal._resetAll.write( 0 );

    _hal._colorEnable.write( 1 );

    tInfo( LOG ) << "System was reset";
}

void SystemManager::setMode( SystemManager::SystemMode mode )
{
    _systemMode = mode;
}

SystemManager::SystemMode SystemManager::mode() const
{
    return _systemMode;
}

const Color& SystemManager::color() const
{
    return _color;
}

//
// Private methods
//
void SystemManager::initRecalage()
{
    if( _color == Color::Yellow )
    {
        _recalage.errorInit( 36, 580, 0 ); // TODO: Change y pos
    }
    else
    {
        _recalage.errorInit( 36, -610, 0 ); // TODO: Change y pos
    }

    tInfo( LOG ) << "Odometry initialized for color:" << _color;
}

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

void SystemManager::displayColor( const DigitalValue& value )
{
    if( value == DigitalValue::OFF )
    {
        _color = Color::Blue;
        _ledBlue->digitalWrite( DigitalValue::ON );
        _ledYellow->digitalWrite( DigitalValue::OFF );
    }
    else
    {
        _color = Color::Yellow;
        _ledBlue->digitalWrite( DigitalValue::OFF );
        _ledYellow->digitalWrite( DigitalValue::ON );
    }
}

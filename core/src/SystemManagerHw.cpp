// Copyright (c) 2019 All Rights Reserved WestBot

#include <QHostAddress>
#include <QThread>

#include <Macros.hpp>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/InputHw.hpp>
#include <WestBot/RobotRock/OdometryHw.hpp>
#include <WestBot/RobotRock/OutputHw.hpp>
#include <WestBot/RobotRock/SystemManagerHw.hpp>
#include <WestBot/RobotRock/TrajectoryManagerHw.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SystemManagerHw" )

    const int GAME_DURATION = 100 * 1000; // 100s
    const QString LIDAR_TOP_TTY = "/dev/ttyAL6";
    const QString LIDAR_FRONT_TTY = "/dev/ttyAL11";
    const QString LIDAR_REAR_TTY = "/dev/ttyAL12";
    const uint32_t LIDAR_BAUDRATE = 256000;
    const int DEFAULT_SIM_PORT = 4242;
}

SystemManagerHw::SystemManagerHw(
    const Hal::Ptr& hal,
    const StrategyManager::Ptr& strategyManager,
    QObject* parent )
    : SystemManager( parent )
    , _hal( hal )
    , _systemMode( SystemManagerHw::SystemMode::Full )

    , _odometry( nullptr )
    , _recalage( nullptr )
    , _lidarTop( nullptr )
    , _lidarFront( nullptr )
    , _lidarRear( nullptr )
    , _trajectoryManager( nullptr )
    , _distanceSensorLeft( "/dev/ttyAL8" )
    , _distanceSensorRight( "/dev/ttyAL13" )
    , _armLeftLow( nullptr )
    , _armRightLow( nullptr )
    , _armLeft( nullptr )
    , _armRight( nullptr )
    , _armsManager( nullptr )
    , _opponentDetection( nullptr )

    , _monitoring( nullptr )
    , _game( nullptr )

    , _strategyManager( strategyManager )
{
    _startButton.reset(
          new InputHw(
              std::make_shared< ItemRegister >( _hal->_input0 ),
             "Tirette" ) );

    _colorButton.reset(
          new InputHw(
              std::make_shared< ItemRegister >( _hal->_input1 ),
              "Color" ) );

    _hardstopButton.reset(
          new InputHw(
              std::make_shared< ItemRegister >( _hal->_input2 ),
              "AU" ) );

    _ledYellow.reset(
          new OutputHw(
              std::make_shared< ItemRegister >( _hal->_output0 ),
              "Yellow" ) );

    _ledBlue.reset(
          new OutputHw(
              std::make_shared< ItemRegister >( _hal->_output2 ),
              "Blue" ) );

    connect(
        _startButton.get(),
        & Input::stateChanged,
        this,
        [ this ]( const DigitalValue& value )
        {
            tDebug( LOG ) << "Start button changed to:" << value;

            if( value == DigitalValue::OFF &&
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
            tDebug( LOG ) << "Color button changed to:" << value;
            displayColor( value );
        } );

    connect(
        _hardstopButton.get(),
        & Input::stateChanged,
        this,
        [ this ]( const DigitalValue& value )
        {
            tDebug( LOG ) << "Hardstop button changed to:" << value;

            if( value == DigitalValue::ON )
            {
                tInfo( LOG ) << "Hardstop requested";
                stop();
            }
        } );

    connect(
        _opponentDetection.get(),
        & OpponentDetection::opponentDetected,
        this,
        [ this ]( double x, double y )
        {
            // TODO: calc obstacle shape from center position ???
            _strategyManager->obstacleAt(x, y, x, y );
        } );

    // we need to provide sw control on this motor
    _hal->_motor5Override.write( 1 );

    _lidarTop.reset( new LidarRPLidarA2(
        LIDAR_TOP_TTY,
        LIDAR_BAUDRATE,
        std::make_shared< ItemRegister >( _hal->_motor5Value ) ) );
    if( ! _lidarTop->init() )
    {
        tCritical( LOG ) << "Failed to init/check health of lidar top module";
        return;
    }

    _lidarFront.reset( new LidarRPLidarA2(
        LIDAR_FRONT_TTY,
        LIDAR_BAUDRATE,
        std::make_shared< ItemRegister >( _hal->_motor5Value ) ) );
    if( ! _lidarFront->init() )
    {
        tCritical( LOG ) << "Failed to init/check health of lidar front module";
        return;
    }

    _lidarRear.reset( new LidarRPLidarA2(
        LIDAR_REAR_TTY,
        LIDAR_BAUDRATE,
        std::make_shared< ItemRegister >( _hal->_motor5Value ) ) );
    if( ! _lidarRear->init() )
    {
        tCritical( LOG ) << "Failed to init/check health of lidar rear module";
        return;
    }

    _hal->_colorEnable.write( 0 );

    _hal->_resetAll.write( 1 );

    _hal->clearRegisters();

    _hal->_resetAll.write( 0 );

    _hal->_colorEnable.write( 1 );

    if( ! _simServer.listen( QHostAddress::Any, DEFAULT_SIM_PORT ) )
    {
       tWarning( LOG )
           << "Unable to start the server:"
           << _simServer.errorString();
    }
}

SystemManagerHw::~SystemManagerHw()
{
    stop();
}

//
// Public methods
//

bool SystemManagerHw::init()
{
    tInfo( LOG ) << "System manager initializing...";

    // For arms
    _hal->_motor4Override.write( 1 );
    _hal->_motor5Override.write( 1 );

    _hal->_motor4Value.write( 0 );

    // Config PID Distance
    _hal->_pidDistanceEnable.write( 0 );
    _hal->_pidDistanceOverride.write( 0 );
    _hal->_pidDistanceInverted.write( 0 );
    _hal->_pidDistanceKp.write( ( float ) 2000.0 );
    _hal->_pidDistanceKi.write( ( float ) 0.0 );
    _hal->_pidDistanceKd.write( ( float ) 0.0 );

    // We set it but this is override by the TrajectoryManager
    _hal->_pidDistanceSpeed.write( ( float ) 0.01 );
    _hal->_pidDistanceAcceleration.write( ( float ) 0.0001 );
    _hal->_pidDistanceSaturation.write( 25000 );

    _hal->_pidDistanceTarget.write( _hal->_pidDistancePosition.read< float >() );
    _hal->_pidDistanceEnable.write( 1 );

    // Config PID Angle
    _hal->_pidAngleEnable.write( 0 );
    _hal->_pidAngleOverride.write( 0 );
    _hal->_pidAngleInverted.write( 1 );
    _hal->_pidAngleKp.write( ( float ) 500000.0 );
    _hal->_pidAngleKi.write( ( float ) 0.0 );
    _hal->_pidAngleKd.write( ( float ) 0.0 );

    _hal->_pidAngleSpeed.write( ( float ) 0.0001 );
    _hal->_pidAngleAcceleration.write( ( float ) 0.00000002 );
    _hal->_pidAngleSaturation.write( 25000 );

    _hal->_pidAngleTarget.write( _hal->_pidAnglePosition.read< float >() );
    _hal->_pidAngleEnable.write( 1 );

    // On set nos pointeurs avant toute chose

    _odometry.reset( new OdometryHw( _hal ) );

    _recalage.reset( new Recalage() );

    if( ! _recalage->init( _odometry, ( LidarBase::Ptr ) _lidarFront ) )
    {
        tWarning( LOG ) << "Failed to init recalage module";
        return false;
    }

    _trajectoryManager.reset(
        new TrajectoryManagerHw( _hal ) );

    _trajectoryManager->init();

    _armLeftLow.reset( new ArmLowLevel() );

    ItemRegister::Ptr pidfirstregL = std::make_shared< ItemRegister >( _hal->_pidCustom1FreqHz );
    ItemRegister::Ptr pumpL = std::make_shared< ItemRegister >( _hal->_motor4Value );
    ItemRegister::Ptr valveL = std::make_shared< ItemRegister >( _hal->_output2 );

    if( ! _armLeftLow->init(
        _hal,
        pidfirstregL,
        true,
        pumpL,
        valveL,
        & _distanceSensorLeft,
        _distanceSensorLeft.distancePointer( 0 ),
        SMART_SERVO_DYNAMIXEL,
        4,
        SMART_SERVO_DYNAMIXEL,
        2,
        SMART_SERVO_DYNAMIXEL,
        5,
        true,
        0.0 ) )
    {
        tFatal( LOG ) << "Unable to init arm left low level. Abort";
    }

    _armRightLow.reset( new ArmLowLevel() );

    ItemRegister::Ptr pidfirstregR = std::make_shared< ItemRegister >( _hal->_pidCustom2FreqHz );
    ItemRegister::Ptr pumpR = std::make_shared< ItemRegister >( _hal->_motor5Value );
    ItemRegister::Ptr valveR = std::make_shared< ItemRegister >( _hal->_output1 );

    if( ! _armLeftLow->init(
        _hal,
        pidfirstregR,
        true,
        pumpR,
        valveR,
        & _distanceSensorRight,
        _distanceSensorRight.distancePointer(0),
        SMART_SERVO_DYNAMIXEL,
        6,
        SMART_SERVO_DYNAMIXEL,
        3,
        SMART_SERVO_DYNAMIXEL,
        7,
        false,
        2.0 ) )
    {
        tFatal( LOG ) << "Unable to init arm left low level. Abort";
    }

    _armLeft.reset( new ArmHighLevel() );

    if( ! _armLeft->init( _odometry, _armLeftLow, true ) )
    {
        tFatal( LOG ) << "Unable to init arm left high level. Abort";
    }

    _armRight.reset( new ArmHighLevel() );

    if( ! _armLeft->init( _odometry, _armRightLow, false ) )
    {
        tFatal( LOG ) << "Unable to init arm right high level. Abort";
    }

    _armsManager.reset( new ArmsManager() );

    if( ! _armsManager->init( _odometry, _armLeft, _armRight ) )
    {
        tFatal( LOG ) << "Unable to init arms manager. Abort";
    }

    _opponentDetection.reset( new OpponentDetection() );

    if( ! _strategyManager->init(
        _odometry,
        _recalage,
        _armsManager,
        _opponentDetection,
        _trajectoryManager ) )
    {
        tFatal( LOG ) << "Unable to init strategy manager. Abort";
    }

    _monitoring.reset( new Monitoring( _hal, _odometry, _armsManager ) );

    _monitoring->start();
    _monitoring->setRefreshRate( 250 );

    // Override output registers
    _hal->_outputOverride.write( 0x01010101 );

    displayColor( _colorButton->digitalRead() );

    tInfo( LOG ) << "System manager initialized";

    return true;
}

void SystemManagerHw::start()
{
    if( nullptr != _game && _game->isRunning() )
    {
        tWarning( LOG ) << "Game is already running";
        return;
    }

    tInfo( LOG ) << "System starting...";

    if( ! isSafe() )
    {
        tFatal( LOG )
            << "System not safe to start: Odometry check failed";
    }

    initRecalage();

    displayColor( _colorButton->digitalRead() );

    blinkColorLed();

    _armsManager->setColor( _color == Color::Blue ? true : false );

    _gameTimer.start( GAME_DURATION );
    _gameTimer.setSingleShot( true );

    _game.reset( new GameThread( _strategyManager, _color ) );
    _game->start();

    _experiment.start();
}

void SystemManagerHw::stop()
{
    SystemManager::stop();

    _gameTimer.stop();

    if( nullptr != _game && _game->isRunning() )
    {
        _game->terminate();
    }

    _strategyManager->stop();

    tInfo( LOG ) << "System stopped";

    reset();
}

void SystemManagerHw::reset()
{
    _hal->_colorEnable.write( 0 );

    _hal->_resetAll.write( 1 );

    _hal->clearRegisters();

    _hal->_resetAll.write( 0 );

    _hal->_colorEnable.write( 1 );

    _monitoring->terminate();
    _monitoring = nullptr;

    _opponentDetection = nullptr;
    _armsManager = nullptr;
    _armRight = nullptr;
    _armLeft = nullptr;
    _armRightLow = nullptr;
    _armLeftLow = nullptr;
    _trajectoryManager = nullptr;
    _recalage = nullptr;
    _odometry = nullptr;

    tInfo( LOG ) << "System was reset";

    init();
}

void SystemManagerHw::setMode( SystemManagerHw::SystemMode mode )
{
    _systemMode = mode;
}

SystemManagerHw::SystemMode SystemManagerHw::mode() const
{
    return _systemMode;
}

bool SystemManagerHw::isSafe() const
{
    // ODOMETRY check
    int16_t x = _odometry->getPosition().x;
    int16_t y = _odometry->getPosition().y;
    int16_t theta = _odometry->getPosition().theta;

    tDebug( LOG ) << "X:" << x << " Y:" << y << " Theta:" << theta;

    int16_t safe = x + y + theta;

    if( safe != 0 )
    {
        return false;
    }

    return true;
}

//
// Private methods
//
void SystemManagerHw::initRecalage()
{
    if( _color == Color::Yellow )
    {
        _odometry->setPosition({.x=0, .y=0, .theta=0});
    }
    else
    {
        _odometry->setPosition({.x=0, .y=0, .theta=0});
    }

    tInfo( LOG ) << "Odometry initialized for color:" << _color << _odometry->getPosition().x
                 << _odometry->getPosition().y << _odometry->getPosition().theta;
}

void SystemManagerHw::displayColor( const DigitalValue& value )
{
    if( value == DigitalValue::OFF )
    {
        _color = Color::Blue;
        _ledBlue->digitalWrite( DigitalValue::ON );
        _ledYellow->digitalWrite( DigitalValue::OFF );
        _experiment.setColorPurple();
    }
    else
    {
        _color = Color::Yellow;
        _ledBlue->digitalWrite( DigitalValue::OFF );
        _ledYellow->digitalWrite( DigitalValue::ON );
        _experiment.setColorYellow();
    }

    _monitoring->updateColor( _color );
}

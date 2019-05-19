// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>
#include <QList>

#include <Macros.hpp>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/InputSimu.hpp>
#include <WestBot/RobotRock/OdometrySimu.hpp>
#include <WestBot/RobotRock/OutputSimu.hpp>
#include <WestBot/RobotRock/SystemManagerSimu.hpp>
#include <WestBot/RobotRock/TrajectoryManagerSimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SystemManagerSimu" )

    const int GAME_DURATION = 100 * 1000; // 100s
    const int DEFAULT_SIM_PORT = 4242;
}

SystemManagerSimu::SystemManagerSimu(
    const StrategyManager::Ptr& strategyManager,
     QObject* parent )
    : SystemManager( parent )
    , _odometry( nullptr )
    //, _recalage( nullptr )
    //, _lidar( nullptr )
    , _trajectoryManager( nullptr )
    , _strategyManager( strategyManager )
    //, _monitoring( nullptr )
    , _game( nullptr )
{
    _color = Color::Yellow;

    _startButton.reset( new InputSimu( 'T', "Tirette" ) );

    _colorButton.reset( new InputSimu( 'C', "Color" ) );

    _hardstopButton.reset( new InputSimu( 'S', "AU" ) );

    _ledYellow.reset( new OutputSimu( 'Y', "Yellow" ) );

    _ledBlue.reset( new OutputSimu( 'B', "Blue" ) );

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

    static bool toggleAvoid = false;

    // TODO: remove this.
    // Test the avoidance
    connect(
        & _opponentTimer,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            //toggleAvoid = ! toggleAvoid;
            //_strategyManager->obstacleToClose( toggleAvoid );

            // On test le serveur tcp
            SimData _data;
            RobotPos pos = _odometry->getPosition();
            _data.objectId = 0;
            _data.objectPos.x = pos.x;
            _data.objectPos.y = pos.y;
            _data.objectPos.theta = pos.theta;
            _data.objectType = 0;
            _data.objectColor = 0;
            _data.objectSize = 100.0;
            _data.objectMode = 0;

            _simServer.updateClients( _data );
        } );

    connect(
        & _simServer,
        & SimTcpServer::onClientConnected,
        this,
        [ this ]( const SimTcpServer::SocketPtr& socket )
        {
            // LEFT SIDE
            static int id = 0;
            {
            SimData _data;
            _data.objectId = id++;
            _data.objectPos.x = -1 * ( 1500 - 500 );
            _data.objectPos.y = 450;
            _data.objectPos.theta = 0;
            _data.objectType = 1;
            _data.objectColor = 0;
            _data.objectSize = 100.0;
            _data.objectMode = 0;

            SimData _data1;
            _data1.objectId = id++;
            _data1.objectPos.x = -1 * ( 1500 - 500 );
            _data1.objectPos.y = 750;
            _data1.objectPos.theta = 0;
            _data1.objectType = 1;
            _data1.objectColor = 0;
            _data1.objectSize = 100.0;
            _data1.objectMode = 0;

            SimData _data2;
            _data2.objectId = id++;
            _data2.objectPos.x = -1 * ( 1500 - 500 );
            _data2.objectPos.y = 1050;
            _data2.objectPos.theta = 0;
            _data2.objectType = 1;
            _data2.objectColor = 1;
            _data2.objectSize = 100.0;
            _data2.objectMode = 0;

            SimData _data3;
            _data3.objectId = id++;
            _data3.objectPos.x = -1 * ( 1500 - 1000 );
            _data3.objectPos.y = 900;
            _data3.objectPos.theta = 0;
            _data3.objectType = 1;
            _data3.objectColor = 2;
            _data3.objectSize = 100.0;
            _data3.objectMode = 0;

            SimData _data4;
            _data4.objectId = id++;
            _data4.objectPos.x = -1 * ( 1500 - 1000 );
            _data4.objectPos.y = 900;
            _data4.objectPos.theta = 0;
            _data4.objectType = 1;
            _data4.objectColor = 2;
            _data4.objectSize = 100.0;
            _data4.objectMode = 0;

            SimData _data5;
            _data5.objectId = id++;
            _data5.objectPos.x = -1 * ( 1500 - 900 );
            _data5.objectPos.y = 1050;
            _data5.objectPos.theta = 0;
            _data5.objectType = 1;
            _data5.objectColor = 1;
            _data5.objectSize = 100.0;
            _data5.objectMode = 0;

            SimData _data6;
            _data6.objectId = id++;
            _data6.objectPos.x = -1 * ( 1500 - 1000 );
            _data6.objectPos.y = 1150;
            _data6.objectPos.theta = 0;
            _data6.objectType = 1;
            _data6.objectColor = 0;
            _data6.objectSize = 100.0;
            _data6.objectMode = 0;

            SimData _data7;
            _data7.objectId = id++;
            _data7.objectPos.x = -1 * ( 1500 - 1150 );
            _data7.objectPos.y = 1050;
            _data7.objectPos.theta = 0;
            _data7.objectType = 1;
            _data7.objectColor = 0;
            _data7.objectSize = 100.0;
            _data7.objectMode = 0;

            QList< SimData > _datas;
            //_datas.push_back( data );
            _datas.push_back( _data );
            _datas.push_back( _data1 );
            _datas.push_back( _data2 );
            _datas.push_back( _data3 );
            _datas.push_back( _data4 );
            _datas.push_back( _data5 );
            _datas.push_back( _data6 );
            _datas.push_back( _data7 );

            _simServer.sendSimData( socket, _datas );
            }

            // RIGHT SIDE
            {
            SimData _data;
            _data.objectId = id++;
            _data.objectPos.x = ( 1500 - 500 );
            _data.objectPos.y = 450;
            _data.objectPos.theta = 0;
            _data.objectType = 1;
            _data.objectColor = 0;
            _data.objectSize = 100.0;
            _data.objectMode = 0;

            SimData _data1;
            _data1.objectId = id++;
            _data1.objectPos.x = ( 1500 - 500 );
            _data1.objectPos.y = 750;
            _data1.objectPos.theta = 0;
            _data1.objectType = 1;
            _data1.objectColor = 0;
            _data1.objectSize = 100.0;
            _data1.objectMode = 0;

            SimData _data2;
            _data2.objectId = id++;
            _data2.objectPos.x = ( 1500 - 500 );
            _data2.objectPos.y = 1050;
            _data2.objectPos.theta = 0;
            _data2.objectType = 1;
            _data2.objectColor = 1;
            _data2.objectSize = 100.0;
            _data2.objectMode = 0;

            SimData _data3;
            _data3.objectId = id++;
            _data3.objectPos.x = ( 1500 - 1000 );
            _data3.objectPos.y = 900;
            _data3.objectPos.theta = 0;
            _data3.objectType = 1;
            _data3.objectColor = 2;
            _data3.objectSize = 100.0;
            _data3.objectMode = 0;

            SimData _data4;
            _data4.objectId = id++;
            _data4.objectPos.x = ( 1500 - 1000 );
            _data4.objectPos.y = 900;
            _data4.objectPos.theta = 0;
            _data4.objectType = 1;
            _data4.objectColor = 2;
            _data4.objectSize = 100.0;
            _data4.objectMode = 0;

            SimData _data5;
            _data5.objectId = id++;
            _data5.objectPos.x = ( 1500 - 900 );
            _data5.objectPos.y = 1050;
            _data5.objectPos.theta = 0;
            _data5.objectType = 1;
            _data5.objectColor = 1;
            _data5.objectSize = 100.0;
            _data5.objectMode = 0;

            SimData _data6;
            _data6.objectId = id++;
            _data6.objectPos.x = ( 1500 - 1000 );
            _data6.objectPos.y = 1150;
            _data6.objectPos.theta = 0;
            _data6.objectType = 1;
            _data6.objectColor = 0;
            _data6.objectSize = 100.0;
            _data6.objectMode = 0;

            SimData _data7;
            _data7.objectId = id++;
            _data7.objectPos.x = ( 1500 - 1150 );
            _data7.objectPos.y = 1050;
            _data7.objectPos.theta = 0;
            _data7.objectType = 1;
            _data7.objectColor = 0;
            _data7.objectSize = 100.0;
            _data7.objectMode = 0;

            QList< SimData > _datas;
            //_datas.push_back( data );
            _datas.push_back( _data );
            _datas.push_back( _data1 );
            _datas.push_back( _data2 );
            _datas.push_back( _data3 );
            _datas.push_back( _data4 );
            _datas.push_back( _data5 );
            _datas.push_back( _data6 );
            _datas.push_back( _data7 );

            _simServer.sendSimData( socket, _datas );
            }
        } );

    connect(
        & _simServer,
        & SimTcpServer::startStrat,
        this,
        [ this ]()
        {
            start();
        } );

    connect(
        & _simServer,
        & SimTcpServer::changeColor,
        this,
        [ this ]( DigitalValue value )
        {
            tDebug( LOG ) << "Color button changed to:" << value;
            displayColor( value );
        } );

    connect(
        & _simServer,
        & SimTcpServer::stop,
        this,
        [ this ]()
        {
            stop();
        } );


    //_lidar.reset( new LidarRPLidarA2(
    //    LIDAR_TTY,
    //    LIDAR_BAUDRATE,
    //    std::make_shared< ItemRegister >( _hal->_motor5Value ) ) );
    //if( ! _lidar->init() )
    //{
    //    tCritical( LOG ) << "Failed to init/check health of lidar module";
    //    return;
    //}

    _opponentTimer.setSingleShot( false );
    _opponentTimer.setInterval( 100 );

    if( ! _simServer.listen( QHostAddress::Any, DEFAULT_SIM_PORT ) )
    {
       tWarning( LOG )
           << "Unable to start the server:"
           << _simServer.errorString();
    }
}

SystemManagerSimu::~SystemManagerSimu()
{
    stop();
}

//
// Public methods
//

bool SystemManagerSimu::init()
{
    tInfo( LOG ) << "System manager initializing...";

    // On set nos pointeurs avant toute chose
    _odometry.reset( new OdometrySimu() );

    //_recalage.reset( new Recalage() );

    //if( ! _recalage->init( _odometry, ( LidarBase::Ptr ) _lidar ) )
    //{
    //    tWarning( LOG ) << "Failed to init recalage module";
    //    return false;
    //}

    _trajectoryManager.reset( new TrajectoryManagerSimu( _odometry ) );

    _trajectoryManager->init();

    if( ! _strategyManager->init( _trajectoryManager ) )
    {
        tFatal( LOG ) << "Unable to init strategy manager. Abort";
    }

    //_monitoring.reset( new Monitoring( _hal, _odometry ) );

    //_monitoring->start();
    //_monitoring->setRefreshRate( 250 );

    //_distanceSensor.start();

    //displayColor( _colorButton->digitalRead() );

    tInfo( LOG ) << "System manager initialized";

    return true;
}

void SystemManagerSimu::start()
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

    //displayColor( _colorButton->digitalRead() );

    blinkColorLed();

    _gameTimer.start( GAME_DURATION );
    _gameTimer.setSingleShot( true );

    _game.reset( new GameThread( _strategyManager, _color ) );
    _game->start();

    _opponentTimer.start();

    //_experiment.start();
}

void SystemManagerSimu::stop()
{
    SystemManager::stop();

    _gameTimer.stop();

    if( nullptr != _game && _game->isRunning() )
    {
        _game->terminate();
    }

    _strategyManager->stop();

    _trajectoryManager->stop();

    _strategyManager->deinit();

    tInfo( LOG ) << "System stopped";

    reset();
}

void SystemManagerSimu::reset()
{
    _odometry = nullptr;
    _trajectoryManager = nullptr;
    //_recalage = nullptr;

    //_monitoring->terminate();
    //_monitoring = nullptr;

    tInfo( LOG ) << "System was reset";

    init();
}

bool SystemManagerSimu::isSafe() const
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
void SystemManagerSimu::initRecalage()
{
    if( _color == Color::Yellow )
    {
        _odometry->setPosition({.x=-1180, .y=600, .theta=0});
    }
    else
    {
        _odometry->setPosition({.x=1180, .y=600, .theta=RAD( 180.0 )});
    }

    tInfo( LOG ) << "Odometry initialized for color:" << _color << _odometry->getPosition().x
                 << _odometry->getPosition().y << _odometry->getPosition().theta;
}

void SystemManagerSimu::displayColor( const DigitalValue& value )
{
    if( value == DigitalValue::OFF )
    {
        _color = Color::Blue;
        _ledBlue->digitalWrite( DigitalValue::ON );
        _ledYellow->digitalWrite( DigitalValue::OFF );
        //_experiment.setColorPurple();
    }
    else
    {
        _color = Color::Yellow;
        _ledBlue->digitalWrite( DigitalValue::OFF );
        _ledYellow->digitalWrite( DigitalValue::ON );
        //_experiment.setColorYellow();
    }

    //_monitoring->updateColor( _color );
}

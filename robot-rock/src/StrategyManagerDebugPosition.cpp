// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ActionList.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

#include "StrategyManagerDebugPosition.hpp"

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.StrategyManagerDebugPosition" )


    PuckPos GndStart1 = {.x = 450, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_RED, .isOnGround = true};
    PuckPos GndStart2 = {.x = 750, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_RED, .isOnGround = true};
    PuckPos GndStart3 = {.x = 1050, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_GREEN, .isOnGround = true};

#define MAIN_DISTRI_RETREY_OFFSET 50.0

    PuckPos MainDistri1Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*0), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri2Green    = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*1), .z = 100+28.5, .theta = M_PI, .type = PUCK_GREEN, .isOnGround = false};
    PuckPos MainDistri3Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*2), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri4Blue     = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*3), .z = 100+28.5, .theta = M_PI, .type = PUCK_BLUE, .isOnGround = false};
    PuckPos MainDistri5Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*4), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri6Green    = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-505-100*5), .z = 100+28.5, .theta = M_PI, .type = PUCK_GREEN, .isOnGround = false};


    PuckPos SmallDistr1Blue  = {.x = 2000, .y = (2925-1500-50-100*0), .z = 70+30+28.5, .theta = M_PI, .type = PUCK_BLUE, .isOnGround = false};
    PuckPos SmallDistr2Green = {.x = 2000, .y = (2925-1500-50-100*1), .z = 70+30+28.5, .theta = M_PI, .type = PUCK_GREEN, .isOnGround = false};
    PuckPos SmallDistr3Red   = {.x = 2000, .y = (2925-1500-50-100*2), .z = 70+30+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};

    PuckPos AccBlue = {25, (210), 150+28.5, .theta = 0.0, .type = PUCK_BLUE, .isOnGround = false};

    PuckPos AccGold = {50, (685+80/2), 165+28.5, .theta = M_PI, .type = PUCK_GOLD, .isOnGround = false};


}

/*
typedef struct
{
    double x;
    double y;
    double z;
    double theta; // used when isOnGround = true
    enum PuckType type;
    bool isOnGround;
} PuckPos;
*/
// enum PuckType { PUCK_UNKNOWN, PUCK_RED, PUCK_GREEN, PUCK_BLUE, PUCK_GOLD};


/*
Violet = Normal
Jaune = Inverted

GroundRed1 {450, (2500-1500)*inv, 25)};
GroundRed2 {750, (2500-1500)*inv, 25)};
GroundGreen {1050, (2500-1500)*inv, 25)};

MainDistri1Red  {1543, (2550-1500-50-100*0)*inv, 100+28.5)};
MainDistri2Green  {1543, (2550-1500-50-100*1)*inv, 100+28.5)};
MainDistri3Red  {1543, (2550-1500-50-100*2)*inv, 100+28.5)};
MainDistri4Blue {1543, (2550-1500-50-100*3)*inv, 100+28.5)};
MainDistri5Red {1543, (2550-1500-50-100*4)*inv, 100+28.5)};
MainDistri6Green {1543, (2550-1500-50-100*5)*inv, 100+28.5)};

SmallDistr1Blue  {2000, (2925-1500-50-100*0)*inv, 70+30+28.5)};
SmallDistr2Green  {2000, (2925-1500-50-100*1)*inv, 70+30+28.5)};
SmallDistr3Red  {2000, (2925-1500-50-100*2)*inv, 70+30+28.5)};

AccBlue  {25, (210)*inv, 150+28.5)};

AccGold  {50, (685+80/2)*inv, 165+28.5)};
*/

StrategyManagerDebugPosition::StrategyManagerDebugPosition( QObject* parent )
    : _odometry( nullptr )
    , _recalage( nullptr )
    , _armsManager( nullptr )
    , _opponentDetection( nullptr )
    , _trajectoryManager( nullptr )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _obstacleToClose( false )
    , _init( false )
{

    tDebug( LOG ) << "Strategy Hello";

    _invArms = false;

    _puckList << &GndStart1;
    _puckList << &GndStart2;
    _puckList << &GndStart3;

    _puckList << &MainDistri1Red;
    _puckList << &MainDistri2Green;
    _puckList << &MainDistri3Red;
    _puckList << &MainDistri4Blue;
    _puckList << &MainDistri5Red;
    _puckList << &MainDistri6Green;

    _puckList << &SmallDistr1Blue;
    _puckList << &SmallDistr2Green;
    _puckList << &SmallDistr3Red;

    _puckList << &AccBlue;
    _puckList << &AccGold;

}

bool StrategyManagerDebugPosition::init(
    const Odometry::Ptr& odometry,
    const Recalage::Ptr& recalage,
    const ArmsManager::Ptr& armsManager,
    const OpponentDetection::Ptr opponentDetection,
    const TrajectoryManager::Ptr& trajectoryManager )
{
    if( ! _init )
    {
        _odometry = odometry;
        _recalage = recalage;
        _armsManager = armsManager;
        _opponentDetection = opponentDetection;
        _trajectoryManager = trajectoryManager;
        _init = true;
    }
    else
    {
        tDebug( LOG ) << "Already initialized";
    }

    return true;
}

bool StrategyManagerDebugPosition::init(
    const Odometry::Ptr& odometry,
    const TrajectoryManager::Ptr& trajectoryManager )
{
    if( ! _init )
    {
        _odometry = odometry;
        _trajectoryManager = trajectoryManager;
        _init = true;
    }
    else
    {
        tDebug( LOG ) << "Already initialized";
    }

    return true;
}

void StrategyManagerDebugPosition::deinit()
{
    _opponentDetection = nullptr;
    _armsManager = nullptr;
    _recalage = nullptr;
    _trajectoryManager = nullptr;
    _odometry = nullptr;
    _init = false;
}

void StrategyManagerDebugPosition::stop()
{
    hardStop();

    // Stop traj
    _trajectoryManager->hardStop();
    _trajectoryManager->disable();

    deinit();
}
/*
PuckPos StrategyManagerHomologation::invertPuckPos(PuckPos* left, PuckPos *right, bool invert)
{
    if (invert)
        return right;
    else
        return left;
}*/

// Private methods
void StrategyManagerDebugPosition::buildStrat( const Color& color )
{
    float inv = 1.0;
    float shift = 0.0;
    float offset = 0.0;

    //RobotPos initPos = {.x = 600.0, .y = 1270.0, .theta = -M_PI/2};
    RobotPos initPos = {.x = 600.0+75.0, .y = 1500.0-450+52+172.8, .theta = -M_PI/2};


    if( color == Color::Yellow )
    {
        inv = -1.0;
        shift = 0.0;
        offset = 0.0;
        _invArms = true;
    }
    else
    {
        inv = 1.0;
        shift = 0.0;
        offset = 0.0;
        _invArms = false;

    }

    if (inv == -1.0)
    {
        initPos.y *= inv;
        initPos.theta+=M_PI;
    }

    for (int i = 0; i < _puckList.size(); i++)
    {
        if (inv == -1.0 && _puckList[i]->y >= 0.0)
            _puckList[i]->y *= -1.0;

        if (inv == 1.0 && _puckList[i]->y < 0.0)
            _puckList[i]->y *= -1.0;
    }

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x,
                            (float)initPos.y*-1.0,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x+100.0,
                            (float)initPos.y*-1.0,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x+100.0,
                            (float)initPos.y,
                            true ));
/*
    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
                            90.0,
                            0.0,
                            0,
                            0,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x,
                            (float)initPos.y,
                            true ));



    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
                            DEG((float)90.0),
                            0.0,
                            0,
                            0,
                            true ));



    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x,
                            (float)initPos.y*-1.0,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
                            90.0,
                            0.0,
                            0,
                            0,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)initPos.x,
                            (float)initPos.y,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS,
                            DEG((float)initPos.theta),
                            0.0,
                            0,
                            0,
                            true ));
*/

    _stratIsRunning = true;
    _trajectoryManager->setAbort( false );
}

void StrategyManagerDebugPosition::doStrat( const Color& color )
{
	// Build the strat for selected color
    buildStrat( color );

    tDebug( LOG ) << "Do strat for color:" << color;

    while( _stratIsRunning )
    {
        if( _obstacleToClose )
        {
            _trajectoryManager->hardStop();
            _trajectoryManager->setAbort(false);

            _actions.push_front( wait500Ms() );
        }

        if( _actions.size() != 0 )
        {
            tDebug(LOG) << _odometry->getPosition().x << _odometry->getPosition().y << DEG(_odometry->getPosition().theta);
            _currentAction = _actions.takeFirst();
            _currentAction->execute();
        }
        else
        {
            tDebug( LOG ) << "NO MORE ACTIONS: Wait until the end";
            _actions.push_front( wait500Ms() );
        }
    }
}

void StrategyManagerDebugPosition::hardStop()
{
    _trajectoryManager->setAbort( true );
    _stratIsRunning = false;
    _actions.clear();
}


void StrategyManagerDebugPosition::obstacleAt( double xStart, double yStart, double xEnd, double yEnd )
{
    /*_obstacleToClose = avoid;

    if( _obstacleToClose )
    {
        _trajectoryManager->setAbort( true );
    }*/
}

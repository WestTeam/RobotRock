// Copyright (c) 2019 All Rights Reserved WestBot

#include <iostream>

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ActionList.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

#include "StrategyManagerMatch3.hpp"

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.StrategyManagerHomologation" )


    PuckPos GndStart1 = {.x = 450, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_RED, .isOnGround = true};
    PuckPos GndStart2 = {.x = 750, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_RED, .isOnGround = true};
    PuckPos GndStart3 = {.x = 1050, .y = (2500-1500), .z = 25, .theta = 0.0, .type = PUCK_GREEN, .isOnGround = true};

#define MAIN_DISTRI_RETREY_OFFSET 0.0

    PuckPos MainDistri1Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*0), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri2Green    = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*1), .z = 100+28.5, .theta = M_PI, .type = PUCK_GREEN, .isOnGround = false};
    PuckPos MainDistri3Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*2), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri4Blue     = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*3), .z = 100+28.5, .theta = M_PI, .type = PUCK_BLUE, .isOnGround = false};
    PuckPos MainDistri5Red      = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*4), .z = 100+28.5, .theta = M_PI, .type = PUCK_RED, .isOnGround = false};
    PuckPos MainDistri6Green    = {.x = 1543-MAIN_DISTRI_RETREY_OFFSET, .y = (2550-1500-50-100*5), .z = 100+28.5, .theta = M_PI, .type = PUCK_GREEN, .isOnGround = false};


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

StrategyManagerMatch3::StrategyManagerMatch3( QObject* parent )
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

bool StrategyManagerMatch3::init(
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

bool StrategyManagerMatch3::init(
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

void StrategyManagerMatch3::deinit()
{
    _opponentDetection = nullptr;
    _armsManager = nullptr;
    _recalage = nullptr;
    _trajectoryManager = nullptr;
    _odometry = nullptr;
    _init = false;
}

void StrategyManagerMatch3::stop()
{
    hardStop();

    // Stop traj
    _trajectoryManager->hardStop();
    _trajectoryManager->disable();

    deinit();
}

// Private methods
void StrategyManagerMatch3::buildStrat( const Color& color )
{
    float inv = 1.0;
    float shift = 0.0;
    float offset = 0.0;

    RobotPos initPos = {.x = 600.0, .y = 1270.0, .theta = -M_PI/2};

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

    RobotPos posScale;

    RobotPos posCatchGndPuck3 = initPos;

    PuckPos *left = nullptr;
    PuckPos *right = &GndStart3;

    if( color == Color::Yellow )
    {
        left = right;
        right = nullptr;
    }

    _armsManager->getCatchPosition(left,nullptr,right,nullptr,posCatchGndPuck3);
    _armsManager->getReleaseScalePosition(posScale);

#define TEST_ONLY_DISTRI_ON_SIDE

#ifndef TEST_ONLY_DISTRI_ON_SIDE

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &GndStart2,
                nullptr,
                &GndStart1,
                nullptr,
                _invArms
                ));

    _actions.push_back( std::make_shared< MoveAction >(
        _trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        (float)posCatchGndPuck3.x,
        (float)posCatchGndPuck3.y,
        true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &GndStart3,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

    RobotPos posBeforeDepose = {.x = 900.0,.y = (1500-450.0/2.0)*inv, .theta = 0.0};

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)posBeforeDepose.x,
                            (float)posBeforeDepose.y,
                            true ));

    RobotPos posDepose = posBeforeDepose;

    PuckPos fakePuckLine;
    fakePuckLine.isOnGround = true;
    fakePuckLine.x = 600;
    fakePuckLine.y = (1500-450.0/2.0)*inv;

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
                            0.0,
                            0.0,
                            (float)fakePuckLine.x,
                            (float)fakePuckLine.y,
                            true ));

    _armsManager->getCatchPosition(&fakePuckLine,nullptr,nullptr,nullptr,posDepose);

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)posDepose.x,
                            (float)posDepose.y,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
                            0.0,
                            0.0,
                            (float)fakePuckLine.x,
                            (float)fakePuckLine.y,
                            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::RELEASE_ALL_PUCKS_GROUND,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                false
                ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)1350.0,
                            (float)posDepose.y,
                            true ));

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
            0.0,
            0.0,
            (float)1350.0,
            inv * 1000.0,
            true ));

#define PUCK_DISTRI_OFFSET_ROBOT_X (230.0)
#define PUCK_DISTRI_OFFSET_ROBOT_Y (250.0*inv)

    // CHECK PUCK 1

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)(MainDistri1Red.x+PUCK_DISTRI_OFFSET_ROBOT_X),
            (float)(MainDistri1Red.y+PUCK_DISTRI_OFFSET_ROBOT_Y),
            true ));

#endif

#ifdef TEST_ONLY_DISTRI_ON_SIDE

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP1,
                &MainDistri1Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));
#endif

    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );



    // Get first puck

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP2,
                &MainDistri1Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));





    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3,
                &MainDistri1Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP1,
                &MainDistri1Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));


#ifndef TEST_ONLY_DISTRI_ON_SIDE

    // CHECK PUCK 2

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)1350.0,
            (2550-1500-50-100*1)*inv,
            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &MainDistri2Green,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

    // CHECK PUCK 3

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)1350.0,
            (2550-1500-50-100*2)*inv,
            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &MainDistri3Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));


    // On va depose le tout
    _actions.push_back( std::make_shared< MoveAction >(
        _trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        (float)posBeforeDepose.x,
        (float)posBeforeDepose.y,
        true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)posDepose.x,
                            (float)posDepose.y,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
                            0.0,
                            0.0,
                            (float)fakePuckLine.x,
                            (float)fakePuckLine.y,
                            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::RELEASE_ALL_PUCKS_GROUND,
                nullptr,
                nullptr,
                nullptr,
                nullptr,
                false
                ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)1350.0,
                            (float)posDepose.y,
                            true ));

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
            0.0,
            0.0,
            (float)1350.0,
            inv * 1000.0,
            true ));

    // CHECK PUCK 4

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)1350.0,
            (2550-1500-50-100*3)*inv,
            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &MainDistri4Blue,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

    // CHECK PUCK 5

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)1350.0,
            (2550-1500-50-100*4)*inv,
            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &MainDistri5Red,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

    // CHECK PUCK 6

    _actions.push_back(
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (float)1350.0,
            (2550-1500-50-100*5)*inv,
            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                &MainDistri6Green,
                nullptr,
                nullptr,
                nullptr,
                _invArms
                ));

#endif

    // ON DROP DANS LA BALANCE
    // TODO: XXX

    /*
    left = &MainDistri2Green;
    right = &MainDistri4Blue;

    if( color == Color::Yellow )
    {
        left = right;
        right = &MainDistri2Green;
    }

    RobotPos distriPos;

    _armsManager->getCatchPosition(left,nullptr,right,nullptr,distriPos);

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)distriPos.x,
                            (float)distriPos.y,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS,
                            DEG((float)distriPos.theta),
                            0.0,
                            0,
                            0,
                            true ));



    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                left,
                nullptr,
                right,
                nullptr,
                _invArms
                ));


    left->x+=25.0;
    right->x+=25.0;

    _armsManager->getCatchPosition(left,nullptr,right,nullptr,distriPos);

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)distriPos.x,
                            (float)distriPos.y,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS,
                            DEG((float)distriPos.theta),
                            0.0,
                            0,
                            0,
                            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_AND_STORE,
                left,
                nullptr,
                right,
                nullptr,
                _invArms
                ));


    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_STEP1,
                left,
                nullptr,
                right,
                nullptr,
                _invArms
                ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)distriPos.x+30.0,
                            (float)distriPos.y,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS,
                            DEG((float)distriPos.theta),
                            0.0,
                            0,
                            0,
                            true ));

    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)distriPos.x-100.0,
                            (float)distriPos.y,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
                            0.0,
                            0.0,
                            (float)900.0,
                            (float)900.0*inv,
                            true ));


    _actions.push_back( std::make_shared< MoveAction >(
                            _trajectoryManager,
                            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
                            0.0,
                            0.0,
                            (float)900.0,
                            (float)1000.0*inv,
                            true ));

    _actions.push_back(
        std::make_shared< ArmsManagerAction >(
                _armsManager,
                ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_STEP2,
                left,
                nullptr,
                right,
                nullptr,
                _invArms
                ));
*/

    _stratIsRunning = true;
    _trajectoryManager->setAbort( false );
}

void StrategyManagerMatch3::doStrat( const Color& color )
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

void StrategyManagerMatch3::hardStop()
{
    _trajectoryManager->setAbort( true );
    _stratIsRunning = false;
    _actions.clear();
}


void StrategyManagerMatch3::obstacleAt( double xStart, double yStart, double xEnd, double yEnd )
{
    /*_obstacleToClose = avoid;

    if( _obstacleToClose )
    {
        _trajectoryManager->setAbort( true );
    }*/
}

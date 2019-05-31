// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmsManagerAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmManagerAction" )
}

ArmsManagerAction::ArmsManagerAction(
    const ArmsManager::Ptr& armsManager,
    Type type,
    PuckPos *left1,
    PuckPos *left2,
    PuckPos *right1,
    PuckPos *right2,
    bool invArms//if true, left is assigned to right
        )
    : Action( "ArmsManagerAction" )
    , _armsManager( armsManager )
    , _type( type )
    , _left1( left1 )
    , _left2( left2 )
    , _right1( right1 )
    , _right2( right2 )
{
    _invArms = invArms;
    if (invArms)
    {
        _left1 = right1;
        _left2 = right2;
        _right1 = left1;
        _right2 = left2;
    }
}


void ArmsManagerAction::execute()
{
    tDebug( LOG ) << "Running" << name() << "action";

    double inv = 1.0;
    double dist = 0.0;

    if (_invArms)
        inv = -1.0;

    switch( _type )
    {
    case ArmsManagerAction::Type::SAFETY_CHECK:
        break;

    case ArmsManagerAction::Type::INIT_POSITION:
        _armsManager->_arm[0]->moveZ(150.0);
        _armsManager->_arm[1]->moveZ(150.0);
        break;

    case ArmsManagerAction::Type::GET_PUCKS:
        _armsManager->getPucks(_left1,_right1);
        break;

    case ArmsManagerAction::Type::GET_PUCKS_AND_STORE:
        _armsManager->getPucksAndStore(_left1,_left2,_right1,_right2);
        break;

    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_STEP1:
        _armsManager->_arm[0]->setMode(ARM_HL_MODE_VERTICAL);
        _armsManager->_arm[1]->setMode(ARM_HL_MODE_VERTICAL);

        _armsManager->_arm[0]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,-22.0);
        _armsManager->_arm[0]->_armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,+22.0);
        _armsManager->_arm[1]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,22.0);
        _armsManager->_arm[1]->_armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,-22.0);

        _armsManager->_arm[0]->_armLL->waitServosTargetOk(1000);
        _armsManager->_arm[1]->_armLL->waitServosTargetOk(1000);


        _armsManager->_arm[0]->disable();
        _armsManager->_arm[1]->disable();
        _armsManager->_arm[0]->setVacuum(true);
        _armsManager->_arm[1]->setVacuum(true);
        break;

    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_STEP2:
        _armsManager->_arm[0]->enable();
        _armsManager->_arm[1]->enable();

        _armsManager->_arm[0]->setMode(ARM_HL_MODE_HORIZONTAL);
        _armsManager->_arm[1]->setMode(ARM_HL_MODE_HORIZONTAL);

        _armsManager->_arm[0]->moveArmRel(320.0,-100);
        _armsManager->_arm[1]->moveArmRel(320.0,100);

        _armsManager->_arm[0]->moveZ(85);
        _armsManager->_arm[1]->moveZ(85);

        _armsManager->_arm[0]->setVacuum(false);
        _armsManager->_arm[1]->setVacuum(false);

         //320.0
        // -100.0

        //_armsManager->_arm[0]->_armLL[]


        break;



    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_STEP3:
        _armsManager->getPucksAndStore(_left1,_left2,_right1,_right2);
        break;


        // prepare (arm not used safe mode, arm used replié mais pret)
    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP1:


        _armsManager->_arm[_invArms]->setMode(ARM_HL_MODE_HORIZONTAL);
        _armsManager->_arm[!_invArms]->setMode(ARM_HL_MODE_VERTICAL);

        _armsManager->_arm[_invArms]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,0.0);
        _armsManager->_arm[_invArms]->_armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,-45.0*inv);
        _armsManager->_arm[!_invArms]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,60.0*inv);
        _armsManager->_arm[!_invArms]->_armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,-90.0*inv);

        _armsManager->_arm[_invArms]->_armLL->waitServosTargetOk(1000);
        _armsManager->_arm[!_invArms]->_armLL->waitServosTargetOk(1000);

        _armsManager->_arm[_invArms]->moveZ(160.0);
        _armsManager->_arm[!_invArms]->moveZ(135.0);


        //_armsManager->_arm[0]->disable();
        //_armsManager->_arm[1]->disable();

        _armsManager->_arm[!_invArms]->setVacuum(true);
        break;

    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP2:
        _armsManager->_arm[!_invArms]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,0.0);

        _armsManager->_arm[!_invArms]->_armLL->waitServosTargetOk(1000);

        QThread::msleep(2000);
        _armsManager->_arm[!_invArms]->moveZ(135.0+40.0);

        break;


    case ArmsManagerAction::Type::GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3:
        _armsManager->_arm[!_invArms]->_armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,60.0*inv);
        _armsManager->_arm[!_invArms]->_armLL->waitServosTargetOk(1000);

        dist = _armsManager->_arm[!_invArms]->getObjectDistance();

        tDebug(LOG) << "GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3: dist: " << dist;
        if (_armsManager->_arm[!_invArms]->getObjectDistance() < 20.0)
        {
            tDebug(LOG) << "GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3: OK! Puck Attached, we can store it";
            _armsManager->_arm[!_invArms]->actionPuckStore();
        } else {
            tDebug(LOG) << "GET_PUCKS_ON_DISTRI_ON_SIDE_STEP3: not OK, no puck Attached";
         }

        _armsManager->_arm[!_invArms]->setVacuum(false);
        break;


    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_ACCELERATOR:
        _armsManager->releasePucksAcceletator();

        break;

    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_SCALE:
        _armsManager->releasePucksScale();
        break;

    case ArmsManagerAction::Type::RELEASE_ALL_PUCKS_GROUND:
        _armsManager->releasePucksGround();

        break;

    default:
        tWarning( LOG ) << "Not a valid ArmsManager action type:" << _type;
    }

    emit complete();
}

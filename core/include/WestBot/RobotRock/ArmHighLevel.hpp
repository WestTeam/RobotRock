// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_
#define WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_

#include <array>
#include <mutex>          // std::mutex, std::unique_lock
#include <queue>
#include <QMutex>

#include <QString>

#include "Hal.hpp"

#include <WestBot/RobotRock/Odometry.hpp>
#include <WestBot/RobotRock/ArmLowLevel.hpp>


namespace WestBot {
namespace RobotRock {

enum ArmHighLevelStorage { ARM_HL_STORAGE_LEFT, ARM_HL_STORAGE_RIGHT };
enum ArmHighLevelMode {
        ARM_HL_MODE_HORIZONTAL, // head is pointing the ground
        ARM_HL_MODE_VERTICAL  // head is in the same alignment as the lower arm
};


typedef struct
{
    double x; // mm
    double y; // mm
    double z; // mm
} Pos3D;

class ArmHighLevel
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmHighLevel" );

    ArmHighLevel();
    ~ArmHighLevel();

    bool init(
         const Odometry::Ptr& odometry, const ArmLowLevelBase::Ptr& armLL
    );

    bool isAttached() const;

    // general
    void disable();
    void enable();

    // position of the ARM axe compared to robot center
    void confArmPos(double xMm,double yMm);
    void confStorage(enum ArmHighLevelStorage id, double xMm,double yMm, double zMm);

    // move Arm to absolute position
    bool moveArmAbs(double xMm, double yMm);

    // move Arm to relative position from robot center
    bool moveArmRel(double xMm, double yMm);

    // move Z (take into account the current H/V mode)
    void moveZ(double ZMm);

    void setMode(enum ArmHighLevelMode mode);

    void setVacuum(bool enable);

    // absolute value of detected object
    void getObjectPos(double &xMm, double &yMm, double &zMm);

    double getObjectDistance();

    uint8_t getPuckCount(enum ArmHighLevelStorage id);

    ///// ACTION /////
    bool actionSafePosition();
    bool actionGroundPuckCollection(double xMm, double yMm);
    bool actionDistributorPuckCollection(double xMm, double yMm);
    bool actionCheckGoldDoorOpen(double xMm, double yMm);
    bool actionGoldPuckCollection(double xMm, double yMm);
    bool actionPuckStore(enum ArmHighLevelStorage id);
    bool actionPuckUnstore(enum ArmHighLevelStorage id);
    bool actionPuckRelease(double xMm, double yMm, double zMm);

private:

    bool _attached;
    //bool _initOk;

    ArmHighLevelMode _mode;

    Odometry::Ptr _odometry;
    ArmLowLevelBase::Ptr _armLL;

    RobotPos _armPos;
    Pos3D _storagePos[2];
    uint8_t _storagePuckCount[2];


};

}
}

#endif // WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_

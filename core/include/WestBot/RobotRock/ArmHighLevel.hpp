// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_
#define WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_

#include <array>
#include <memory>
#include <mutex>          // std::mutex, std::unique_lock
#include <queue>

#include <QMutex>
#include <QString>

#include "ArmLowLevel.hpp"
#include "Hal.hpp"
#include "Odometry.hpp"

#include <WestBot/HumanAfterAll/Category.hpp>



int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime);




int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime);


namespace WestBot {
namespace RobotRock {

enum ArmHighLevelStorage { ARM_HL_STORAGE_LEFT, ARM_HL_STORAGE_RIGHT };
enum ArmHighLevelMode {
        ARM_HL_MODE_HORIZONTAL, // head is pointing the ground
        ARM_HL_MODE_VERTICAL  // head is in the same alignment as the lower arm
};

enum PuckType { PUCK_UNKNOWN, PUCK_RED, PUCK_GREEN, PUCK_BLUE, PUCK_GOLD};

typedef struct
{
    double x; // mm
    double y; // mm
    double z; // mm
} Pos3D;

#define WRIST_AND_SUCTION_LENGTH 35.0
#define ARM_UPPER_LENGTH 70.0
#define ARM_LOWER_LENGTH 70.0
#define DISTRI_HEIGHT 100.0
#define PUCK_DIAMETER 76.0
#define GOLD_CHECKPOINT_Z (165.0+PUCK_DIAMETER/2)
#define PUCK_WIDTH 25.0


class ArmHighLevel
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmHighLevel" );
    using Ptr = std::shared_ptr< ArmHighLevel >;

    ArmHighLevel();
    ~ArmHighLevel();

    bool init(
         const Odometry::Ptr& odometry, const ArmLowLevelBase::Ptr& armLL, bool isLeft
    );

    bool isAttached() const;

    // general
    void disable();
    void enable();

    // position of the ARM axe compared to robot center
    void confArmPos(double xMm,double yMm);
    void confStorage(double xMm,double yMm, double zMm);

    void getArmPos(double &xMm, double &yMm);

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

    uint8_t getPuckCount();

    ///// ACTION /////
    bool actionSafePosition();
    bool actionGroundPuckCollection(double xMm, double yMm);
    bool actionDistributorPuckCollection(double xMm, double yMm);
    bool actionCheckGoldDoorOpen(double xMm, double yMm);
    bool actionGoldPuckCollection(double xMm, double yMm);
    bool actionPuckStore();
    bool actionPuckUnstore();
    bool actionPuckRelease(double xMm, double yMm, double zMm);

private:
    bool _attached;
    //bool _initOk;

    bool _isLeft;

    ArmHighLevelMode _mode;

    Odometry::Ptr _odometry;
    ArmLowLevelBase::Ptr _armLL;

    RobotPos _armPos;
    Pos3D _storagePos;
    uint8_t _storagePuckCount;

    bool _vaccumEnalbed;


};

}
}

#endif // WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_

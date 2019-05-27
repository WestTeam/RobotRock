// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARMS_MANAGER_HPP_
#define WESTBOT_ROBOTROCK_ARMS_MANAGER_HPP_

#include <array>
#include <mutex>          // std::mutex, std::unique_lock
#include <queue>
#include <QMutex>
#include <memory>

#include <QString>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Odometry.hpp>
#include <WestBot/RobotRock/ArmHighLevel.hpp>

#include "Hal.hpp"

namespace WestBot {
namespace RobotRock {

/*
enum ArmHighLevelStorage { ARM_HL_STORAGE_LEFT, ARM_HL_STORAGE_RIGHT };
enum ArmHighLevelMode {
        ARM_HL_MODE_HORIZONTAL, // head is pointing the ground
        ARM_HL_MODE_VERTICAL  // head is in the same alignment as the lower arm
};*/


typedef struct
{
    double x;
    double y;
    double z;
    double theta; // used when isOnGround = true
    enum PuckType type;
    bool isOnGround;
} PuckPos;

class ArmsManager
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmsManager" );

    using Ptr = std::shared_ptr< ArmsManager >;

    ArmsManager();
    ~ArmsManager();

    bool init(
         const Odometry::Ptr& odometry, const ArmHighLevel::Ptr& armLeft, const ArmHighLevel::Ptr& armRight
    );



    bool isAttached() const;

    // general
    void disable();
    void enable();

    void setColor(bool isPurple);
    int getScore();

    // get ideal position
    bool getCatchPosition(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight, RobotPos &pos);
    // get a list of pucks for each arm and store each of them (if the last one cannot be stored (full), we keep it outside)
    bool getPucksAndStore(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight);
    void getPucksAndStoreSingle(bool isRight, std::list<PuckPos> *list, bool *ret);


    // if null, no one to take
    bool getPucks(PuckPos *left, PuckPos *right);

    void getPuck(bool isRight, PuckPos *puck, bool *ret);

    void getReleaseAcceleratorPosition(RobotPos &pos);

    // release all stored pucks into the accelerator
    bool releasePucksAcceletator();

    void getReleaseScalePosition(RobotPos &pos);

    // release all stored pucks into the scale
    bool releasePucksScale();

    void getReleaseGroundPosition(RobotPos &pos);
    bool releasePucksGround();



private:

    bool _attached;

    Odometry::Ptr _odometry;
    ArmHighLevel::Ptr _arm[2];

    int _score;
    bool _isPurple;

};

}
}

#endif // WESTBOT_ROBOTROCK_ARM_HIGH_LEVEL_HPP_

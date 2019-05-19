// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARMACTION_HPP_
#define WESTBOT_ROBOTROCK_ARMACTION_HPP_

#include <memory>

#include "Action.hpp"
#include "ArmHighLevel.hpp"

namespace WestBot {
namespace RobotRock {

class ArmAction : public Action
{
public:
    using Ptr = std::shared_ptr< ArmAction >;

    enum Type
    {
        SAFETY_CHECK = 0,
        GROUND_PUCK_COLLECT,
        DISTRIBUTOR_PUCK_COLLECT,
        GOLD_DOOR_OPEN_CHECK,
        GOLD_PUCK_COLLECT,
        PUCK_STORE,
        PUCK_UNSTORE,
        PUCK_RELEASE
    };

    enum AppliedTo
    {
        ARM_LEFT = 0,
        ARM_RIGHT,
        BOTH
    };

    ArmAction(
        const ArmHighLevel::Ptr& armLeft,
        const ArmHighLevel::Ptr& armRight,
        Type type,
        double xMm,
        double yMm,
        double zMm,
        AppliedTo toArm
        );

    void execute() override;

private:
    ArmHighLevel::Ptr _armLeft;
    ArmHighLevel::Ptr _armRight;
    Type _type;
    double _xMm;
    double _yMm;
    double _zMm;
    AppliedTo _toArm;
};

}
}

#endif // WESTBOT_ROBOTROCK_ARMACTION_HPP_


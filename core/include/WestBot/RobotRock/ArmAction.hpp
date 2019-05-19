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

    ArmAction(
        const ArmHighLevel::Ptr& arm,
        Type type,
        double xMm,
        double yMm,
        double zMm
        );

    void execute() override;

private:
    ArmHighLevel::Ptr _arm;
    Type _type;
    double _xMm;
    double _yMm;
    double _zMm;
};

}
}

#endif // WESTBOT_ROBOTROCK_ARMACTION_HPP_


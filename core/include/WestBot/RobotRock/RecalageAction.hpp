// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_RECALAGEACTION_HPP_
#define WESTBOT_ROBOTROCK_RECALAGEACTION_HPP_

#include <memory>

#include "Action.hpp"
#include "Recalage.hpp"

namespace WestBot {
namespace RobotRock {

class RecalageAction : public Action
{
public:
    using Ptr = std::shared_ptr< RecalageAction >;

    RecalageAction(
        const Recalage::Ptr& recalage,
        bool triggerOnce,
        bool continuous
        );

    void execute() override;

private:
     Recalage::Ptr _recalage;
     bool _triggerOnce;
     bool _continuous;
};

}
}

#endif // WESTBOT_ROBOTROCK_RECALAGEACTION_HPP_


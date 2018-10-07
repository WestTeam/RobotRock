// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_WAITACTION_HPP_
#define WESTBOT_ROBOTROCK_WAITACTION_HPP_

#include <memory>

#include "Action.hpp"

namespace WestBot {
namespace RobotRock {

class WaitAction : public Action
{
public:
    using Ptr = std::shared_ptr< WaitAction >;

    WaitAction( int waitMs );

    void execute() override;

private:
    int _waitMs;
};

}
}

#endif // WESTBOT_ROBOTROCK_WAITACTION_HPP_


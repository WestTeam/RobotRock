// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_OUTPUTSIMU_HPP_
#define WESTBOT_ROBOTROCK_OUTPUTSIMU_HPP_

#include "Output.hpp"

namespace WestBot {
namespace RobotRock {

class OutputSimu : public Output
{
public:
    OutputSimu( char key, const QString& name );

    void digitalWrite( DigitalValue val ) override;

    DigitalValue digitalRead() override;

private:
    char _key;
};

}
}

#endif // WESTBOT_ROBOTROCK_OUTPUTSIMU_HPP_

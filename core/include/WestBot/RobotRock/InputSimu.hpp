// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_INPUTSIMU_HPP_
#define WESTBOT_ROBOTROCK_INPUTSIMU_HPP_

#include "Input.hpp"

class QTimer;

namespace WestBot {
namespace RobotRock {

class InputSimu : public Input
{
public:
    InputSimu( char key, const QString& name );

private:
    /*!
     * \brief Check the input state and update it's internal state.
     */
    void check() override;

private:
    char _key;
    QTimer* _eventTimer;
};

}
}

#endif // WESTBOT_ROBOTROCK_INPUTSIMU_HPP_

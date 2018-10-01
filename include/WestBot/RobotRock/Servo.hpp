// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_SERVO_HPP_
#define WESTBOT_ROBOTROCK_SERVO_HPP_

#include <QString>

#include "Hal.hpp"

namespace WestBot {
namespace RobotRock {

class Servo
{
public:
    Servo( const QString& name );

    bool attach(
        Hal& hal,
        uint8_t ioNumber,
        uint16_t min,
        uint16_t max );

    const QString& name() const;

    uint16_t read();
    void write( uint16_t position );

    void enable();
    void disable();

    bool isAttached() const;

private:
    const QString _name;

    ItemRegister::Ptr _servo;
    ItemRegister::Ptr _servoEnable;
    ItemRegister::Ptr _servoOverride;

    uint16_t _minAngle;
    uint16_t _maxAngle;
};

}
}

#endif // WESTBOT_ROBOTROCK_SERVO_HPP_

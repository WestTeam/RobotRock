// Copyright (c) 2018 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Servo.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Servo" )
}

Servo::Servo( const QString& name )
    : _name( name )
    , _servo( nullptr )
    , _servoEnable( nullptr )
    , _servoOverride( nullptr )
    , _minAngle( 0 )
    , _maxAngle( 0 )
{
}

bool Servo::attach(
    Hal& hal,
    uint8_t ioNumber,
    uint16_t min,
    uint16_t max )
{
    switch( ioNumber )
    {
    case 0:
        _servo = std::make_shared< ItemRegister >( hal._s0 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s0Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s0Override );
        break;

    case 1:
        _servo = std::make_shared< ItemRegister >( hal._s1 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s1Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s1Override );
        break;

    case 2:
        _servo = std::make_shared< ItemRegister >( hal._s2 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s2Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s2Override );
        break;

    case 3:
        _servo = std::make_shared< ItemRegister >( hal._s3 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s3Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s3Override );
        break;

    case 4:
        _servo = std::make_shared< ItemRegister >( hal._s4 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s4Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s4Override );
        break;

    case 5:
        _servo = std::make_shared< ItemRegister >( hal._s5 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s5Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s5Override );
        break;

    case 6:
        _servo = std::make_shared< ItemRegister >( hal._s6 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s6Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s6Override );
        break;

    case 7:
        _servo = std::make_shared< ItemRegister >( hal._s7 );
        _servoEnable = std::make_shared< ItemRegister >( hal._s7Enable );
        _servoOverride = std::make_shared< ItemRegister >( hal._s7Override );
        break;

    default:
        tWarning( LOG ) << "Not a valid io number to attach servo.";
        return false;
    }

    _minAngle = min;
    _maxAngle = max;

    _servo->write( min );
    _servoOverride->write( 0x01 );
    _servoEnable->write( 0x03 );

    QThread::msleep( 500 );

    return true;
}

const QString& Servo::name() const
{
    return _name;
}

uint16_t Servo::read()
{
    return _servo->read< uint16_t >();
}

void Servo::write( uint16_t read )
{
    _servo->write( read );
}

void Servo::enable()
{
    _servoEnable->write( 0x03 );
}

void Servo::disable()
{
    _servoEnable->write( 0x00 );
}

bool Servo::isAttached() const
{
    return _servoEnable->read< uint8_t >() == 0x03 ? true : false;
}

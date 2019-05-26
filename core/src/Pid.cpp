// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>
#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Hal.hpp>

#include <WestBot/RobotRock/Pid.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;


Pid::Pid(Memory &layer, int offset)
    : _layer(layer)
      ,_pidFreqHz( _layer, offset+(0)* 4, 16 )
      , _pidFreqHzLatest( _layer, offset+(0) * 4 + 2, 16 )
      , _pidEnable( _layer, offset+(1) * 4, 8 )
      , _pidOverride(  _layer, offset+(1) * 4 + 1, 8 )
      , _pidInverted(  _layer, offset+(1) * 4 + 2, 8 )
      , _pidKp( _layer, offset+(2) * 4, 32 )
      , _pidKi( _layer, offset+(3) * 4, 32 )
      , _pidKd(  _layer, offset+(4) * 4, 32 )
      , _pidSpeed(  _layer, offset+(5) * 4, 32 )
      , _pidAcceleration(  _layer, offset+(6) * 4, 32 )
      , _pidSaturation(  _layer, offset+(7) * 4, 32 )
      , _pidPosition(  _layer, offset+(8) * 4, 32 )
      , _pidTarget( _layer, offset+(9) * 4, 32 )
      , _pidOutput( _layer, offset+(10) * 4, 32 )
      , _pidReference( _layer, offset+(11) * 4, 32 )
{

    setEnable(false);
    /*
     *
    _hal->_pidCustomTarget.write( _hal->_pidCustomPosition.read< int32_t >() );

    // we disable PID during config
    _hal->_pidCustomEnable.write( 0 );

    // we get sw control of the PID
    _hal->_pidCustomOverride.write( 1 );

    // to check if we need to invert or not
    _hal->_pidCustomInverted.write( 0 );

    // we set coefs
    _hal->_pidCustomKp.write( (float)300.0 );
    _hal->_pidCustomKi.write( (float)0.0 );
    _hal->_pidCustomKd.write( (float)0.0 );

    // we set speed, acc and output saturation
    _hal->_pidCustomSpeed.write( (float)40.0 );
    _hal->_pidCustomAcceleration.write( (float)0.010 );
    _hal->_pidCustomSaturation.write( 10000 );
    */

}


Pid::~Pid()
{

    tDebug( LOG ) << "Pid destructor engaged";

    setEnable(false);
}


// if override = false, the following data is configured from upper layer :
// enable
// acceleration
// speed
// target

void Pid::setOverride(bool override)
{
    _pidOverride.write(override);
}
void Pid::setEnable(bool enable)
{
    _pidEnable.write(enable);
}
void Pid::setInverted(bool inverted)
{
    _pidInverted.write(inverted);

}
void Pid::setKp(float kp)
{
    _pidKp.write(kp);
}
void Pid::setKi(float ki)
{
    _pidKi.write(ki);
}
void Pid::setKd(float kd)
{
    _pidKd.write(kd);
}
void Pid::setFreqHz(uint16_t freqhz)
{
    _pidFreqHz.write(freqhz);
}
void Pid::setSaturation(uint32_t sat)
{
    _pidSaturation.write(sat);
}
void Pid::setSpeed(float speed)
{
    _pidSpeed.write(speed);
}
void Pid::setAcceleration(float acc)
{
    _pidAcceleration.write(acc);
}

void Pid::setTarget(int32_t target)
{
    _pidTarget.write(target);
}

void Pid::setReference(int32_t ref)
{
    _pidReference.write(ref);
}


uint16_t Pid::getFreqHzLatest()
{
    return _pidFreqHzLatest.read<uint16_t>();
}
int32_t Pid::getInput()
{
    return _pidPosition.read<int32_t>();
}
int32_t Pid::getOutput()
{
    return _pidOutput.read<int32_t>();

}
int32_t Pid::getReference()
{
    return _pidReference.read<int32_t>();
}




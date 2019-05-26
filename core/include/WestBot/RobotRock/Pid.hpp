// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_PID_HPP_
#define WESTBOT_ROBOTROCK_PID_HPP_


#include <QString>

#include "Hal.hpp"



namespace WestBot {
namespace RobotRock {



class Pid
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Pid" );

    Pid(Memory &layer, int offset);
    ~Pid();

    void setOverride(bool override);
    void setEnable(bool enable);
    void setInverted(bool inverted);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setFreqHz(uint16_t freqhz);
    void setSaturation(uint32_t sat);
    void setSpeed(float speed);
    void setAcceleration(float acc);

    void setTarget(int32_t target);

    void setReference(int32_t ref);

    uint16_t getFreqHzLatest();
    int32_t getInput();
    int32_t getOutput();
    int32_t getReference();

private:
    Memory _layer;
    ItemRegister _pidFreqHz;
    ItemRegister _pidFreqHzLatest;
    ItemRegister _pidEnable;
    ItemRegister _pidOverride;
    ItemRegister _pidInverted;
    ItemRegister _pidKp;
    ItemRegister _pidKi;
    ItemRegister _pidKd;
    ItemRegister _pidSpeed;
    ItemRegister _pidAcceleration;
    ItemRegister _pidSaturation;
    ItemRegister _pidPosition;
    ItemRegister _pidTarget;
    ItemRegister _pidOutput;

    ItemRegister _pidReference;
};

}
}

#endif // WESTBOT_ROBOTROCK_PID_HPP_

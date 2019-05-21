// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_HAL_HPP_
#define WESTBOT_ROBOTROCK_HAL_HPP_

#include <memory>

#include "ItemRegister.hpp"
#include "MemoryManager.hpp"

namespace WestBot {
namespace RobotRock {

class Hal
{
public:
    using Ptr = std::shared_ptr< Hal >;

    Hal();
    ~Hal();

    void dump();

    void clearRegisters();

private:
    MemoryManager _memoryManager;
    Memory _layer1;
    Memory _layer2;
    Memory _layer3;

public:
    // Layer 1
    ItemRegister _resetAll; // Reset for layer 1 / layer 2 / layer 3
    ItemRegister _colorEnable;
    ItemRegister _initOkL1;
    ItemRegister _modeSimu;
    ItemRegister _voltage24V;
    ItemRegister _voltageA12V;
    ItemRegister _voltageA5V;
    ItemRegister _voltage12V;
    ItemRegister _buzzer;
    ItemRegister _buzzerOverride;
    ItemRegister _input0;
    ItemRegister _input1;
    ItemRegister _input2;
    ItemRegister _input3;
    ItemRegister _inputOverride;
    ItemRegister _output0;
    ItemRegister _output1;
    ItemRegister _output2;
    ItemRegister _output3;
    ItemRegister _outputOverride;

    // Servo
    ItemRegister _s0;
    ItemRegister _s0Enable;
    ItemRegister _s0Override;
    ItemRegister _s1;
    ItemRegister _s1Enable;
    ItemRegister _s1Override;
    ItemRegister _s2;
    ItemRegister _s2Enable;
    ItemRegister _s2Override;
    ItemRegister _s3;
    ItemRegister _s3Enable;
    ItemRegister _s3Override;
    ItemRegister _s4;
    ItemRegister _s4Enable;
    ItemRegister _s4Override;
    ItemRegister _s5;
    ItemRegister _s5Enable;
    ItemRegister _s5Override;
    ItemRegister _s6;
    ItemRegister _s6Enable;
    ItemRegister _s6Override;
    ItemRegister _s7;
    ItemRegister _s7Enable;
    ItemRegister _s7Override;

    ItemRegister _colorSensorValid;
    ItemRegister _colorSensorRed;
    ItemRegister _colorSensorGreen;
    ItemRegister _colorSensorBlue;
    ItemRegister _colorSensorClear;

    ItemRegister _distanceSensor;

    ItemRegister _esc0Enable;
    ItemRegister _esc0Override;
    ItemRegister _esc0Value;

    ItemRegister _motor0Override;
    ItemRegister _motor0Inverted;
    ItemRegister _motor0Value;

    ItemRegister _motor1Override;
    ItemRegister _motor1Inverted;
    ItemRegister _motor1Value;

    ItemRegister _motor2Override;
    ItemRegister _motor2Inverted;
    ItemRegister _motor2Value;

    ItemRegister _motor3Override;
    ItemRegister _motor3Inverted;
    ItemRegister _motor3Value;

    ItemRegister _motor4Override;
    ItemRegister _motor4Inverted;
    ItemRegister _motor4Value;

	ItemRegister _motor5Override;
    ItemRegister _motor5Inverted;
	ItemRegister _motor5Value;

    ItemRegister _qei0Override;
    ItemRegister _qei0RefValue;
    ItemRegister _qei0CntValue;

    ItemRegister _qei1Override;
    ItemRegister _qei1RefValue;
    ItemRegister _qei1CntValue;

    ItemRegister _qei2Override;
    ItemRegister _qei2RefValue;
    ItemRegister _qei2CntValue;

    ItemRegister _qei3Override;
    ItemRegister _qei3RefValue;
    ItemRegister _qei3CntValue;

    ItemRegister _qei4Override;
    ItemRegister _qei4RefValue;
    ItemRegister _qei4CntValue;

    ItemRegister _qei5Override;
    ItemRegister _qei5RefValue;
    ItemRegister _qei5CntValue;

    ItemRegister _pwmCustom0Value;
    ItemRegister _pwmCustom1Value;
    ItemRegister _pwmCustom2Value;
    ItemRegister _pwmCustom3Value;


    // General Smart Servo Command
    ItemRegister _smartServoCmdValid;
    ItemRegister _smartServoCmdId;
    ItemRegister _smartServoCmdType;
    ItemRegister _smartServoCmdDevId;
    ItemRegister _smartServoCmdAck;
    ItemRegister _smartServoCmdError;


    // RAW Command definition
    ItemRegister _smartServoCmdRawOnHold;
    ItemRegister _smartServoCmdRawProtocol;
    ItemRegister _smartServoCmdRawInstr;
    ItemRegister _smartServoCmdRawAddr;
    ItemRegister _smartServoCmdRawData;

    // Register Device Command definition
    ItemRegister _smartServoCmdRegisterProtocol;
    ItemRegister _smartServoCmdRegisterBusId;

    // Set Pos Command definition
    ItemRegister _smartServoCmdSetPosOnHold;
    ItemRegister _smartServoCmdSetPosWaitPosition;
    ItemRegister _smartServoCmdSetPosPosition;

    // Set Pos And Speed Command definition
    ItemRegister _smartServoCmdSetPosAndSpeedOnHold;
    ItemRegister _smartServoCmdSetPosAndSpeedWaitPosition;
    ItemRegister _smartServoCmdSetPosAndSpeedPosition;
    ItemRegister _smartServoCmdSetPosAndSpeedSpeed;

    // Set Action Command definition
    ItemRegister _smartServoCmdSetActionWaitPosition;

    // Get Status Command definition
    ItemRegister _smartServoCmdGetStatusPosition;
    ItemRegister _smartServoCmdGetStatusLoad;
    ItemRegister _smartServoCmdGetStatusVoltage;
    ItemRegister _smartServoCmdGetStatusTemp;

    // Set Enable
    ItemRegister _smartServoCmdSetEnableOnHold;
    ItemRegister _smartServoCmdSetEnableEnable;

    // Layer 2
    ItemRegister _odometryFreqHz;
    ItemRegister _odometryFreqHzLatest;
    ItemRegister _odometryAck;
    ItemRegister _odometryCodingWheelAxeMm;
    ItemRegister _odometryCodingWheelMmPerTickLeft;
    ItemRegister _odometryCodingWheelMmPerTickRight;
    ItemRegister _odometryCodingMotorAxeMm;
    ItemRegister _odometryCodingMotorMmPerTickLeft;
    ItemRegister _odometryCodingMotorMmPerTickRight;
    ItemRegister _odometryValid;
    ItemRegister _odometryId;
    ItemRegister _odometryTheta;
    ItemRegister _odometryX;
    ItemRegister _odometryY;

    // PID DISTANCE
    ItemRegister _pidDistanceFreqHz;
    ItemRegister _pidDistanceFreqHzLatest;
    ItemRegister _pidDistanceEnable;
    ItemRegister _pidDistanceOverride;
    ItemRegister _pidDistanceInverted;
    ItemRegister _pidDistanceKp;
    ItemRegister _pidDistanceKi;
    ItemRegister _pidDistanceKd;
    ItemRegister _pidDistanceSpeed;
    ItemRegister _pidDistanceAcceleration;
    ItemRegister _pidDistanceSaturation;
    ItemRegister _pidDistancePosition;
    ItemRegister _pidDistanceTarget;
    ItemRegister _pidDistanceOutput;

    // PID ANGLE
    ItemRegister _pidAngleFreqHz;
    ItemRegister _pidAngleFreqHzLatest;
    ItemRegister _pidAngleEnable;
    ItemRegister _pidAngleOverride;
    ItemRegister _pidAngleInverted;
    ItemRegister _pidAngleKp;
    ItemRegister _pidAngleKi;
    ItemRegister _pidAngleKd;
    ItemRegister _pidAngleSpeed;
    ItemRegister _pidAngleAcceleration;
    ItemRegister _pidAngleSaturation;
    ItemRegister _pidAnglePosition;
    ItemRegister _pidAngleTarget;
    ItemRegister _pidAngleOutput;

    // PID Custom
    ItemRegister _pidCustom1FreqHz;
    ItemRegister _pidCustom1FreqHzLatest;
    ItemRegister _pidCustom1Enable;
    ItemRegister _pidCustom1Override;
    ItemRegister _pidCustom1Inverted;
    ItemRegister _pidCustom1Kp;
    ItemRegister _pidCustom1Ki;
    ItemRegister _pidCustom1Kd;
    ItemRegister _pidCustom1Speed;
    ItemRegister _pidCustom1Acceleration;
    ItemRegister _pidCustom1Saturation;
    ItemRegister _pidCustom1Position;
    ItemRegister _pidCustom1Target;
    ItemRegister _pidCustom1Output;
    ItemRegister _pidCustom1LastReference;

    // PID Custom
    ItemRegister _pidCustom2FreqHz;
    ItemRegister _pidCustom2FreqHzLatest;
    ItemRegister _pidCustom2Enable;
    ItemRegister _pidCustom2Override;
    ItemRegister _pidCustom2Inverted;
    ItemRegister _pidCustom2Kp;
    ItemRegister _pidCustom2Ki;
    ItemRegister _pidCustom2Kd;
    ItemRegister _pidCustom2Speed;
    ItemRegister _pidCustom2Acceleration;
    ItemRegister _pidCustom2Saturation;
    ItemRegister _pidCustom2Position;
    ItemRegister _pidCustom2Target;
    ItemRegister _pidCustom2Output;
    ItemRegister _pidCustom2LastReference;

    // Layer 3
    ItemRegister _trajFreqHz;
    ItemRegister _trajCmdValid;
    ItemRegister _trajCmdId;
    ItemRegister _trajCmdType;
    ItemRegister _trajCmdOrderType;
    ItemRegister _trajCmdWndDistance;
    ItemRegister _trajCmdWndAngleDeg;
    ItemRegister _trajCmdWndAngleStartDeg;
    ItemRegister _trajCmdCfgSpeed;
    ItemRegister _trajCmdCfgAcc;
    ItemRegister _trajCmdPosTheta;
    ItemRegister _trajCmdPosX;
    ItemRegister _trajCmdPosY;
    ItemRegister _trajCmdADAngleDeg;
    ItemRegister _trajCmdADDistance;
    ItemRegister _trajCmdADCorrection;

    ItemRegister _trajOutAck;
    ItemRegister _trajOutState;
    ItemRegister _trajOutBlocked;
    ItemRegister _trajOutInWindow;
};

}
}

#endif // WESTBOT_ROBOTROCK_HAL_HPP_

// Copyright (c) 2018 All Rights Reserved WestBot

#include <hps_arm.h> // For our base address

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Hal.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Hal" )
}

Hal::Hal()
    : _layer1( _memoryManager , PIO_N_LAYER1_BASE )
    , _layer2( _memoryManager, PIO_N_LAYER2_BASE )
    , _layer3( _memoryManager, PIO_N_LAYER3_BASE )
    // Layer 1
    , _resetAll( _layer1, 0 * 4, 8 )
    , _colorEnable( _layer1, 0 * 4 + 1, 8 )
    , _initOkL1( _layer1, 1 * 4, 8 )
    , _modeSimu( _layer1, 1 * 4 + 1, 8 )
    , _voltage24V( _layer1, 2 * 4, 32 )
    , _voltageA12V( _layer1, 3 * 4, 32 )
    , _voltageA5V( _layer1, 4 * 4, 32 )
    , _voltage12V( _layer1, 5 * 4, 32 )
    , _buzzer( _layer1, 6 * 4, 8 )
    , _buzzerOverride( _layer1, 6 * 4 +1, 8 )
    , _input0( _layer1, 7 * 4, 8 )
    , _input1( _layer1, 7 * 4 + 1, 8 )
    , _input2( _layer1, 7 * 4  + 2, 8  )
    , _input3( _layer1, 7 * 4 + 3, 8 )
    , _inputOverride( _layer1, 8 * 4, 32 )
    , _output0( _layer1, 9 * 4, 8 )
    , _output1( _layer1, 9 * 4 + 1, 8 )
    , _output2( _layer1, 9 * 4 + 2, 8 )
    , _output3( _layer1, 9 * 4 + 3, 8 )
    , _outputOverride( _layer1, 10 * 4, 32 )
    , _s0( _layer1, 30 * 4, 16 )
    , _s0Enable( _layer1, 30 * 4 + 2, 8 )
    , _s0Override( _layer1, 30 * 4 + 3, 8 )
    , _s1( _layer1, 31 * 4, 16 )
    , _s1Enable( _layer1, 31 * 4 + 2, 8 )
    , _s1Override( _layer1, 31 * 4 + 3, 8 )
    , _s2( _layer1, 32 * 4, 16 )
    , _s2Enable( _layer1, 32 * 4 + 2, 8 )
    , _s2Override( _layer1, 32 * 4 + 3, 8 )
    , _s3( _layer1, 33 * 4, 16 )
    , _s3Enable( _layer1, 33 * 4 + 2, 8)
    , _s3Override( _layer1, 33 * 4 + 3, 8 )
    , _s4( _layer1, 34 * 4, 16 )
    , _s4Enable( _layer1, 34 * 4 + 2, 8 )
    , _s4Override( _layer1, 34 * 4 + 3, 8 )
    , _s5( _layer1, 35 * 4, 16 )
    , _s5Enable( _layer1, 35 * 4 + 2, 8 )
    , _s5Override( _layer1, 35 * 4 + 3, 8 )
    , _s6( _layer1, 36 * 4, 16 )
    , _s6Enable( _layer1, 36 * 4 + 2, 8 )
    , _s6Override( _layer1, 36 * 4 + 3, 8 )
    , _s7( _layer1, 37 * 4, 16 )
    , _s7Enable( _layer1, 37 * 4 + 2, 8 )
    , _s7Override( _layer1, 37 * 4 + 3, 8 )
    , _colorSensorValid( _layer1, 50 * 4, 32 )
    , _colorSensorRed( _layer1, 51 * 4, 16 )
    , _colorSensorGreen( _layer1, 51 * 4 + 2, 16 )
    , _colorSensorBlue( _layer1, 52 * 4, 16 )
    , _colorSensorClear( _layer1, 52 * 4 + 2, 16 )

    , _distanceSensor( _layer1, 53 * 4, 32 )

    , _esc0Enable( _layer1, 46 * 4 + 2, 8 )
    , _esc0Override( _layer1, 46 * 4 + 3, 8 )
    , _esc0Value( _layer1, 46 * 4, 16 )

    // Motor color orientation
    , _motor3Override( _layer1, 15 * 4 + 3, 8 )
    , _motor3Inverted( _layer1, 15 * 4 + 2, 8 )
    , _motor3Value( _layer1, 15 * 4, 16 )

	// Motor color orientation
	, _motor5Override( _layer1, 17 * 4 + 3, 8 )
	, _motor5Value( _layer1, 17 * 4, 16 )

    // Smart Servo commands
    , _smartServoCmdValid( _layer1, 61 * 4 + 0, 8)
    , _smartServoCmdId( _layer1, 61 * 4 + 1, 8)
    , _smartServoCmdType( _layer1, 61 * 4 + 2, 8)
    , _smartServoCmdDevId( _layer1, 61 * 4 + 3, 8)
    , _smartServoCmdAck( _layer1, 63 * 4 + 2, 8)
    , _smartServoCmdError( _layer1, 63 * 4 + 3, 8)


    , _smartServoCmdRawOnHold( _layer1, 62 * 4 + 0, 8)
    , _smartServoCmdRawProtocol( _layer1, 62 * 4 + 1, 8)
    , _smartServoCmdRawInstr( _layer1, 62 * 4 + 2, 8)
    , _smartServoCmdRawAddr( _layer1, 62 * 4 + 3, 8)
    , _smartServoCmdRawData( _layer1, 63 * 4 + 0, 16)


    , _smartServoCmdRegisterProtocol( _layer1, 62 * 4 + 0, 8)
    , _smartServoCmdRegisterBusId( _layer1, 62 * 4 + 1, 8)

    , _smartServoCmdSetPosOnHold( _layer1, 62 * 4 + 0, 8)
    , _smartServoCmdSetPosWaitPosition( _layer1, 62 * 4 + 1, 8)
    , _smartServoCmdSetPosPosition( _layer1, 62 * 4 + 2, 16)

    , _smartServoCmdSetPosAndSpeedOnHold( _layer1, 62 * 4 + 0, 8)
    , _smartServoCmdSetPosAndSpeedWaitPosition( _layer1, 62 * 4 + 1, 8)
    , _smartServoCmdSetPosAndSpeedPosition( _layer1, 62 * 4 + 2, 16)
    , _smartServoCmdSetPosAndSpeedSpeed( _layer1, 63 * 4 + 0, 16)

    , _smartServoCmdSetActionWaitPosition( _layer1, 62 * 4 + 0, 8)

    , _smartServoCmdGetStatusPosition( _layer1, 62 * 4 + 0, 16)
    , _smartServoCmdGetStatusLoad( _layer1, 62 * 4 + 2, 16)
    , _smartServoCmdGetStatusVoltage( _layer1, 63 * 4 + 0, 8)
    , _smartServoCmdGetStatusTemp( _layer1, 63 * 4 + 1, 8)

    , _smartServoCmdSetEnableOnHold( _layer1, 62 * 4 + 0, 8)
    , _smartServoCmdSetEnableEnable( _layer1, 62 * 4 + 1, 8)



    // Layer 2
    , _odometryTheta( _layer2, 13 * 4 + 2, 16 )
    , _odometryX( _layer2, 14 * 4, 16 )
    , _odometryY( _layer2, 14 * 4 + 2, 16 )

    // PID DISTANCE
    , _pidDistanceEnable( _layer2, 16 * 4, 8 )
    , _pidDistanceOverride( _layer2, 16 * 4 + 1, 8 )
    , _pidDistanceInverted( _layer2, 16 * 4 + 2, 8 )
    , _pidDistanceKp( _layer2, 17 * 4, 32 )
    , _pidDistanceKi( _layer2, 18 * 4, 32 )
    , _pidDistanceKd( _layer2, 19 * 4, 32 )
    , _pidDistanceSpeed( _layer2, 20 * 4, 32 )
    , _pidDistanceAcceleration( _layer2, 21 * 4, 32 )
    , _pidDistanceSaturation( _layer2, 22 * 4, 32 )
    , _pidDistancePosition( _layer2, 23 * 4, 32 )
    , _pidDistanceTarget( _layer2, 24 * 4, 32 )

    // PID ANGLE
    , _pidAngleEnable( _layer2, 27 * 4, 8 )
    , _pidAngleOverride( _layer2, 27 * 4 + 1, 8 )
    , _pidAngleInverted( _layer2, 27 * 4 + 2, 8 )
    , _pidAngleKp( _layer2, 28 * 4, 32 )
    , _pidAngleKi( _layer2, 29 * 4, 32 )
    , _pidAngleKd( _layer2, 30 * 4, 32 )
    , _pidAngleSpeed( _layer2, 31 * 4, 32 )
    , _pidAngleAcceleration( _layer2, 32 * 4, 32 )
    , _pidAngleSaturation( _layer2, 33 * 4, 32 )
    , _pidAnglePosition( _layer2, 34 * 4, 32 )
    , _pidAngleTarget( _layer2, 35 * 4, 32 )

    // Layer 3
    , _trajFreqHz( _layer3, 2 * 4, 16 )
    , _trajCmdValid( _layer3, 11 * 4, 8 )
    , _trajCmdId( _layer3, 11 * 4 + 1, 8 )
    , _trajCmdType( _layer3, 11 * 4 + 2, 8 )
    , _trajCmdOrderType( _layer3, 11 * 4 + 3, 8 )
    , _trajCmdWndDistance( _layer3, 12 * 4, 32 )
    , _trajCmdWndAngleDeg( _layer3, 13 * 4, 32 )
    , _trajCmdWndAngleStartDeg( _layer3, 14 * 4, 32 )
    , _trajCmdCfgSpeed( _layer3, 12 * 4, 32 )
    , _trajCmdCfgAcc( _layer3, 13 * 4, 32 )
    , _trajCmdPosTheta( _layer3, 12 * 4, 32 )
    , _trajCmdPosX( _layer3, 13 * 4, 32 )
    , _trajCmdPosY( _layer3, 14 * 4, 32 )
    , _trajCmdADAngleDeg( _layer3, 12 * 4, 32 )
    , _trajCmdADDistance( _layer3, 13 * 4, 32 )
    , _trajCmdADCorrection( _layer3, 14 * 4, 8 )
    , _trajOutAck( _layer3, 15 * 4, 8 )
    , _trajOutState( _layer3, 15 * 4 + 1, 8 )
    , _trajOutBlocked( _layer3, 15 * 4 + 2, 8 )
    , _trajOutInWindow( _layer3, 15 * 4 + 3, 8 )
{
}

Hal::~Hal()
{
    tDebug( LOG ) << "Hal Destroyed";
    _layer1.reset();
    _layer2.reset();
    _layer3.reset();
}

void Hal::dump()
{
    // Read layer 1 registers
    tDebug( LOG ) << "Layer 1 registers:";
    tDebug( LOG ) << "==================";

    for( int i = 0; i < 64; ++i )
    {
        tLogStream( LOG, DebugLevel )
            << "Read register" << i << ":"
            << QString::number( _layer1.read( i * 4, 32 ), 16 );
    }

    // Read layer 2 registers
    tDebug( LOG ) << "Layer 2 registers:";
    tDebug( LOG ) << "==================";

    for( int i = 0; i < 64; ++i )
    {
        tDebug( LOG )
            << "Read register" << i << ":"
            << QString::number( _layer2.read( i * 4, 32 ), 16 );
    }

    // Read layer 3 registers
    tDebug( LOG ) << "Layer 3 registers:";
    tDebug( LOG ) << "==================";

    for( int i = 0; i < 64; ++i )
    {
        tDebug( LOG )
            << "Read register" << i << ":"
            << QString::number( _layer3.read( i * 4, 32 ), 16 );
    }
}

void Hal::clearRegisters()
{
    for( int i = 0; i < 64; i++ )
    {
        _layer2.write( i * 4, 32, 0 );
    }

    for( int i = 0; i < 64; i++ )
    {
        _layer3.write( i * 4, 32, 0 );
    }
}

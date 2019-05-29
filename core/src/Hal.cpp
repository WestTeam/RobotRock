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


    // MOTOR LEFT
    , _motor0Override( _layer1, (12+0) * 4 + 3, 8 )
    , _motor0Inverted( _layer1, (12+0) * 4 + 2, 8 )
    , _motor0Value( _layer1, (12+0) * 4, 16 )

    // MOTOR RIGHT
    , _motor1Override( _layer1, (12+1) * 4 + 3, 8 )
    , _motor1Inverted( _layer1, (12+1) * 4 + 2, 8 )
    , _motor1Value( _layer1, (12+1) * 4, 16 )

    // Motor Custom
    , _motor2Override( _layer1, (12+2) * 4 + 3, 8 )
    , _motor2Inverted( _layer1, (12+2) * 4 + 2, 8 )
    , _motor2Value( _layer1, (12+2) * 4, 16 )

    , _motor3Override( _layer1, (12+3) * 4 + 3, 8 )
    , _motor3Inverted( _layer1, (12+3) * 4 + 2, 8 )
    , _motor3Value( _layer1, (12+3) * 4, 16 )

    , _motor4Override( _layer1, (12+4) * 4 + 3, 8 )
    , _motor4Inverted( _layer1, (12+4) * 4 + 2, 8 )
    , _motor4Value( _layer1, (12+4) * 4, 16 )

    , _motor5Override( _layer1, (12+5) * 4 + 3, 8 )
    , _motor5Inverted( _layer1, (12+5) * 4 + 2, 8 )
    , _motor5Value( _layer1, (12+5) * 4, 16 )


    , _motor0Current( _layer1, (18+0) * 4, 32 )
    , _motor1Current( _layer1, (18+1) * 4, 32 )
    , _motor2Current( _layer1, (18+2) * 4, 32 )
    , _motor3Current( _layer1, (18+3) * 4, 32 )
    , _motor4Current( _layer1, (18+4) * 4, 32 )
    , _motor5Current( _layer1, (18+5) * 4, 32 )

    , _qei0Override( _layer1, (24+0) * 4 + 3, 8 )
    , _qei0RefValue( _layer1, (24+0) * 4 + 2, 8 )
    , _qei0CntValue( _layer1, (24+0) * 4 + 0, 16 )

    , _qei1Override( _layer1, (24+1) * 4 + 3, 8 )
    , _qei1RefValue( _layer1, (24+1) * 4 + 2, 8 )
    , _qei1CntValue( _layer1, (24+1) * 4 + 0, 16 )

    , _qei2Override( _layer1, (24+2) * 4 + 3, 8 )
    , _qei2RefValue( _layer1, (24+2) * 4 + 2, 8 )
    , _qei2CntValue( _layer1, (24+2) * 4 + 0, 16 )

    , _qei3Override( _layer1, (24+3) * 4 + 3, 8 )
    , _qei3RefValue( _layer1, (24+3) * 4 + 2, 8 )
    , _qei3CntValue( _layer1, (24+3) * 4 + 0, 16 )

    , _qei4Override( _layer1, (24+4) * 4 + 3, 8 )
    , _qei4RefValue( _layer1, (24+4) * 4 + 2, 8 )
    , _qei4CntValue( _layer1, (24+4) * 4 + 0, 16 )

    , _qei5Override( _layer1, (24+5) * 4 + 3, 8 )
    , _qei5RefValue( _layer1, (24+5) * 4 + 2, 8 )
    , _qei5CntValue( _layer1, (24+5) * 4 + 0, 16 )

    , _pwmCustom0Value( _layer1, (48) * 4 + 2*0, 16 )
    , _pwmCustom1Value( _layer1, (48) * 4 + 2*1, 16 )
    , _pwmCustom2Value( _layer1, (48) * 4 + 2*2, 16 )
    , _pwmCustom3Value( _layer1, (48) * 4 + 2*3, 16 )

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
    , _odometryFreqHz( _layer2, 2 * 4 + 0, 16 ) // write only
    , _odometryFreqHzLatest( _layer2, 2 * 4 + 0, 16 )
    , _odometryAck( _layer2, 2 * 4 + 0, 8 ) // share the same register as _odometryFreqHz
    , _odometryCodingWheelAxeMm( _layer2, 3 * 4 + 0, 32 )
    , _odometryCodingWheelMmPerTickLeft( _layer2, 4 * 4 + 0, 32 )
    , _odometryCodingWheelMmPerTickRight( _layer2, 5 * 4 + 0, 32 )
    , _odometryCodingMotorAxeMm( _layer2, 6 * 4 + 0, 32 )
    , _odometryCodingMotorMmPerTickLeft( _layer2, 7 * 4 + 0, 32 )
    , _odometryCodingMotorMmPerTickRight( _layer2, 8 * 4 + 0, 32 )
    , _odometryValid( _layer2, 13 * 4 + 0, 8 )
    , _odometryId( _layer2, 13 * 4 + 1, 8 )
    , _odometryTheta( _layer2, 13 * 4 + 2, 16 )
    , _odometryX( _layer2, 14 * 4, 16 )
    , _odometryY( _layer2, 14 * 4 + 2, 16 )

    // PID DISTANCE
    , _pidDistanceFreqHz( _layer2, 15 * 4, 16 )
    , _pidDistanceFreqHzLatest( _layer2, 15 * 4 + 2, 16 )
    , _pidDistanceEnable( _layer2, 16 * 4, 8 )
    , _pidDistanceOverride( _layer2, 16 * 4 + 1, 8 )
    , _pidDistanceInverted( _layer2, 16 * 4 + 2, 8 )
    , _pidDistanceSpeedReductor( _layer2, 16 * 4 + 3, 8 )
    , _pidDistanceKp( _layer2, 17 * 4, 32 )
    , _pidDistanceKi( _layer2, 18 * 4, 32 )
    , _pidDistanceKd( _layer2, 19 * 4, 32 )
    , _pidDistanceSpeed( _layer2, 20 * 4, 32 )
    , _pidDistanceAcceleration( _layer2, 21 * 4, 32 )
    , _pidDistanceSaturation( _layer2, 22 * 4, 32 )
    , _pidDistancePosition( _layer2, 23 * 4, 32 )
    , _pidDistanceTarget( _layer2, 24 * 4, 32 )
    , _pidDistanceOutput( _layer2, 25 * 4, 32 )


    // PID ANGLE
    , _pidAngleFreqHz( _layer2, 26 * 4, 16 )
    , _pidAngleFreqHzLatest( _layer2, 26 * 4 + 2, 16 )
    , _pidAngleEnable( _layer2, 27 * 4, 8 )
    , _pidAngleOverride( _layer2, 27 * 4 + 1, 8 )
    , _pidAngleInverted( _layer2, 27 * 4 + 2, 8 )
    , _pidAngleSpeedReductor( _layer2, 27 * 4 + 3, 8 )
    , _pidAngleKp( _layer2, 28 * 4, 32 )
    , _pidAngleKi( _layer2, 29 * 4, 32 )
    , _pidAngleKd( _layer2, 30 * 4, 32 )
    , _pidAngleSpeed( _layer2, 31 * 4, 32 )
    , _pidAngleAcceleration( _layer2, 32 * 4, 32 )
    , _pidAngleSaturation( _layer2, 33 * 4, 32 )
    , _pidAnglePosition( _layer2, 34 * 4, 32 )
    , _pidAngleTarget( _layer2, 35 * 4, 32 )
    , _pidAngleOutput( _layer2, 36 * 4, 32 )


    // PID Custom
    , _pidCustom1FreqHz( _layer2, (37+0) * 4, 16 )
    , _pidCustom1FreqHzLatest( _layer2, (37+0) * 4 + 2, 16 )
    , _pidCustom1Enable( _layer2, (37+1) * 4, 8 )
    , _pidCustom1Override(  _layer2, (37+1) * 4 + 1, 8 )
    , _pidCustom1Inverted(  _layer2, (37+1) * 4 + 2, 8 )
    , _pidCustom1Kp( _layer2, (37+2) * 4, 32 )
    , _pidCustom1Ki( _layer2, (37+3) * 4, 32 )
    , _pidCustom1Kd(  _layer2, (37+4) * 4, 32 )
    , _pidCustom1Speed(  _layer2, (37+5) * 4, 32 )
    , _pidCustom1Acceleration(  _layer2, (37+6) * 4, 32 )
    , _pidCustom1Saturation(  _layer2, (37+7) * 4, 32 )
    , _pidCustom1Position(  _layer2, (37+8) * 4, 32 )
    , _pidCustom1Target( _layer2, (37+9) * 4, 32 )
    , _pidCustom1Output( _layer2, (37+10) * 4, 32 )
    , _pidCustom1LastReference( _layer2, (37+11) * 4, 32 )

    , _pidCustom2FreqHz( _layer2, (49+0) * 4, 16 )
    , _pidCustom2FreqHzLatest( _layer2, (49+0) * 4 + 2, 16 )
    , _pidCustom2Enable( _layer2, (49+1) * 4, 8 )
    , _pidCustom2Override(  _layer2, (49+1) * 4 + 1, 8 )
    , _pidCustom2Inverted(  _layer2, (49+1) * 4 + 2, 8 )
    , _pidCustom2Kp( _layer2, (49+2) * 4, 32 )
    , _pidCustom2Ki( _layer2, (49+3) * 4, 32 )
    , _pidCustom2Kd(  _layer2, (49+4) * 4, 32 )
    , _pidCustom2Speed(  _layer2, (49+5) * 4, 32 )
    , _pidCustom2Acceleration(  _layer2, (49+6) * 4, 32 )
    , _pidCustom2Saturation(  _layer2, (49+7) * 4, 32 )
    , _pidCustom2Position(  _layer2, (49+8) * 4, 32 )
    , _pidCustom2Target( _layer2, (49+9) * 4, 32 )
    , _pidCustom2Output( _layer2, (49+10) * 4, 32 )
    , _pidCustom2LastReference( _layer2, (49+11) * 4, 32 )

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

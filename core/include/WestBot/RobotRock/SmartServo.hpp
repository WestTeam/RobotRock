// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_SMART_SERVO_HPP_
#define WESTBOT_ROBOTROCK_SMART_SERVO_HPP_

#include <array>
#include <mutex>          // std::mutex, std::unique_lock
#include <queue>
#include <QMutex>


#include <QString>

#include "Hal.hpp"

namespace WestBot {
namespace RobotRock {

#define SMART_SERVO_FEETECH 0
#define SMART_SERVO_DYNAMIXEL 1

//FEETECH : Definition of commands
#define FEETECH_CMD_PING        0x01
#define FEETECH_CMD_READ        0x02
#define FEETECH_CMD_WRITE       0x03
#define FEETECH_CMD_REG_WRITE   0x04
#define FEETECH_CMD_ACTION      0x05
#define FEETECH_CMD_RESET       0x06
#define FEETECH_CMD_SYNC_WRITE  0x83

//DYNAMIXEL : Definition of commands
#define DYNAMIXEL_CMD_PING          0x01
#define DYNAMIXEL_CMD_READ          0x02
#define DYNAMIXEL_CMD_WRITE         0x03
#define DYNAMIXEL_CMD_REG_WRITE     0x04
#define DYNAMIXEL_CMD_ACTION        0x05
#define DYNAMIXEL_CMD_FACTORY_RESET 0x06
#define DYNAMIXEL_CMD_REBOOT        0x08
#define DYNAMIXEL_CMD_STATUS        0x55
#define DYNAMIXEL_CMD_SYNC_READ     0x82
#define DYNAMIXEL_CMD_SYNC_WRITE    0x83
#define DYNAMIXEL_CMD_BULK_READ     0x92
#define DYNAMIXEL_CMD_BULK_WRITE    0x93

//FEETECH : Definition of regs
// EEPROM
#define FEETECH_REGS_ID 0x05
#define FEETECH_REGS_BAUD_RATE 0x06
#define FEETECH_REGS_RETURN_DELAY_TIME 0x07
#define FEETECH_REGS_ANSWER_LEVEL_STATUS 0x08
#define FEETECH_REGS_MIN_ANGLE_LIMIT_H 0x09

#define FEETECH_REGS_MAX_ANGLE_LIMIT_H 0x0b

#define FEETECH_REGS_MAX_TEMP 0x0d
#define FEETECH_REGS_HIGHEST_VOLTAGE 0x0e
#define FEETECH_REGS_LOWEST_VOLTAGE 0x0f
#define FEETECH_REGS_MAX_TORQUE_H 0x10

#define FEETECH_REGS_HIGH_VOLTAGE_FLAG 0x12
#define FEETECH_REGS_UNLOAD_CONDITION 0x13
#define FEETECH_REGS_LED_ALARAM_CONDITION 0x14
#define FEETECH_REGS_P 0x15
#define FEETECH_REGS_I 0x16
#define FEETECH_REGS_D 0x17
#define FEETECH_REGS_MIN_PWM_H 0x18

// RAM
#define FEETECH_REGS_TORQUE_SWITCH 0x28
#define FEETECH_REGS_TARGET_POS_H 0x2a
#define FEETECH_REGS_RUNNING_TIME_H 0x2c
#define FEETECH_REGS_RUNNING_SPEED_H 0x2e
#define FEETECH_REGS_LOCK_SIGN 0x30

#define FEETECH_REGS_CURRENT_POS_H 0x38
#define FEETECH_REGS_CURRENT_SPEED_H 0x3a
#define FEETECH_REGS_CURRENT_LOAD_H 0x3c
#define FEETECH_REGS_CURRENT_VOLTAGE 0x3e
#define FEETECH_REGS_CURRENT_TEMP 0x3f
#define FEETECH_REGS_REG_WRITE_SIGN 0x40

//DYNAMIXEL : Definition of regs
// EEPROM
#define DYNAMIXEL_REGS_ID (3)
#define DYNAMIXEL_REGS_BAUD_RATE (4)
#define DYNAMIXEL_REGS_RETURN_DELAY_TIME (5)
#define DYNAMIXEL_REGS_CW_ANGLE_LIMIT_L (6)
#define DYNAMIXEL_REGS_CCW_ANGLE_LIMIT_L (8)
#define DYNAMIXEL_REGS_CONTROL_MODE (11)
#define DYNAMIXEL_REGS_LIMIT_TEMP (12)
#define DYNAMIXEL_REGS_LOWER_LIMIT_VOLTAGE (13)
#define DYNAMIXEL_REGS_UPPER_LIMIT_VOLTAGE (14)
#define DYNAMIXEL_REGS_MAX_TORQUE_L (15)
#define DYNAMIXEL_REGS_RETURN_LEVEL (17)
#define DYNAMIXEL_REGS_ALARM_SHUTDOWN (18)

// RAM
#define DYNAMIXEL_REGS_TORQUE_ENABLE (24)
#define DYNAMIXEL_REGS_LED (25)
#define DYNAMIXEL_REGS_D (27)
#define DYNAMIXEL_REGS_I (28)
#define DYNAMIXEL_REGS_P (29)
#define DYNAMIXEL_REGS_GOAL_POSITION_L (30)
#define DYNAMIXEL_REGS_GOAL_VELOCITY_L (32)
#define DYNAMIXEL_REGS_GOAL_TORQUE_L (35)
#define DYNAMIXEL_REGS_PRESENT_POSITION_L (37)
#define DYNAMIXEL_REGS_PRESENT_SPEED_L (39)
#define DYNAMIXEL_REGS_PRESENT_LOAD_L (41)
#define DYNAMIXEL_REGS_PRESENT_VOLTAGE (45)
#define DYNAMIXEL_REGS_PRESENT_TEMP (46)
#define DYNAMIXEL_REGS_REGISTERED_INSTR (47)
#define DYNAMIXEL_REGS_MOVING (49)
#define DYNAMIXEL_REGS_HW_ERROR_STATUS (50)
#define DYNAMIXEL_REGS_PUNCH (51)

// Definition of Commands
#define CMD_TYPE_RAW 0
#define CMD_TYPE_REGISTER_DEV 1
#define CMD_TYPE_SET_POS 2
#define CMD_TYPE_SET_POS_AND_SPEED 3
#define CMD_TYPE_SET_ACTION 4
#define CMD_TYPE_GET_STATUS 5
#define CMD_TYPE_SET_ENABLE 6

// Definition of RAW instructions
#define RAW_WR8B 0
#define RAW_WR16B 1
#define RAW_RD8B 2
#define RAW_RD16B 3
#define RAW_ACTION 4

#define HW_SERVO_COUNT 3



class SmartServo;

class SmartServoStaticData
{
public:
    std::array< SmartServo*, HW_SERVO_COUNT > _deviceList = { nullptr };
    std::queue< uint8_t > _qId;
    std::mutex _mutex;
    QMutex _qmutex;

     SmartServoStaticData()
     {
         for( int i = 0; i < HW_SERVO_COUNT; ++i )
         {
            _qId.push( i );
         }
     }
};




class SmartServo
{
public:
    SmartServo( const QString& name );
    ~SmartServo();

    bool attach(
        Hal& hal,
        uint8_t protocol,
        uint8_t busId // id on the hw bus
    );

    const QString& name() const;

    void enable();
    void disable();

    bool isAttached() const;

    void setRawWrite8( uint8_t addr, uint8_t data );
    void setRawWrite16( uint8_t addr, uint16_t data );
    uint8_t getRawRead8( uint8_t addr );
    uint16_t getRawRead16( uint8_t addr );
    void setRawAction();

    void changeId( uint8_t newId );

    void setPosition( bool onHold, bool waitPosition, uint16_t pos );
    void setPositionAndSpeed(
        bool onHold,
        bool waitPosition,
        uint16_t pos,
        uint16_t speed );

    void setAction( bool waitPosition );
    void checkStatus();

    uint16_t getPosition( bool Update );
    uint16_t getLoad( bool Update );
    uint8_t getVoltage( bool Update );
    uint8_t getTemp( bool Update );

    bool moving();

    void setEnable( bool onHold, bool enable );

    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SmartServo" );

private:
    void registerDevice();

private:
    static SmartServoStaticData _staticData;
    static std::mutex _cmdMutex;
    static QMutex _qcmdMutex;

    const QString _name;

    bool _attached;
    Hal _hal;
    uint8_t _protocol;
    uint8_t _busId; // id on the hw bus
    uint8_t _devId;

    uint16_t _current_pos;
    uint16_t _current_load;
    uint8_t _current_voltage;
    uint8_t _current_temp;
};

}
}

#endif // WESTBOT_ROBOTROCK_SERVO_HPP_

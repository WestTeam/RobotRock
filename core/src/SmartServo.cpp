// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SmartServo.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SmartServo" )
}

SmartServoStaticData SmartServo::_staticData;
std::mutex SmartServo::_cmdMutex;

#define CMD_ACK_SLEEP 2

SmartServo::SmartServo( const QString& name )
    : _name( name )
{
}

SmartServo::~SmartServo()
{
    std::unique_lock<std::mutex> lck(_staticData._mutex);

    disable();
    _staticData._qId.push(_devId);
    _staticData._deviceList[_devId] = nullptr;
    tInfo( LOG ) << _devId << "is pushed back";
}


bool SmartServo::attach(
        Hal& hal,
        uint8_t protocol,
        uint8_t busId // id on the hw bus
        /*uint8_t devId*/  )// internal device id (0 to 7)
{
    bool ret = false;
    _protocol = protocol;
    _busId = busId;

    //_devId = devId;

    std::unique_lock<std::mutex> lck(_staticData._mutex);

    if (!_staticData._qId.empty())
    {
        _hal = hal;
        _devId = _staticData._qId.front();
        _staticData._qId.pop();
        _staticData._deviceList[_devId] = this;
        tInfo( LOG ) << _devId << _protocol << _busId;

        registerDevice();
        ret = true;
    }

    lck.unlock();


    return ret;
}

void SmartServo::registerDevice()
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    // REG DEVICE
    _hal._smartServoCmdValid.write(0);
    tInfo( LOG ) << "reg device command" << _protocol << _busId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_REGISTER_DEV);
    _hal._smartServoCmdDevId.write(_devId);
    _hal._smartServoCmdRegisterProtocol.write(_protocol); // dyna
    _hal._smartServoCmdRegisterBusId.write(_busId);

    _hal._smartServoCmdValid.write(1);

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "reg device wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}


void SmartServo::setRawWrite8(uint8_t addr, uint8_t data)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    tInfo( LOG ) << "raw write8 command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_RAW);
    _hal._smartServoCmdDevId.write(_busId);

    _hal._smartServoCmdRawOnHold.write(0);
    _hal._smartServoCmdRawProtocol.write(_protocol);
    _hal._smartServoCmdRawInstr.write(RAW_WR8B);
    _hal._smartServoCmdRawAddr.write(addr);//0x2a);
    _hal._smartServoCmdRawData.write(data);

    _hal._smartServoCmdValid.write(1);

    //tInfo( LOG ) << "exec command";

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}
void SmartServo::setRawWrite16(uint8_t addr, uint16_t data)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    tInfo( LOG ) << "raw write16 command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_RAW);
    _hal._smartServoCmdDevId.write(_busId);

    _hal._smartServoCmdRawOnHold.write(0);
    _hal._smartServoCmdRawProtocol.write(_protocol);
    _hal._smartServoCmdRawInstr.write(RAW_WR16B);
    _hal._smartServoCmdRawAddr.write(addr);
    _hal._smartServoCmdRawData.write(data);

    _hal._smartServoCmdValid.write(1);

    //tInfo( LOG ) << "exec command";

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}
uint8_t SmartServo::setRawRead8(uint8_t addr)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "raw read8 command" << _devId << addr;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_RAW);
    _hal._smartServoCmdDevId.write(_busId);

    _hal._smartServoCmdRawOnHold.write(0);
    _hal._smartServoCmdRawProtocol.write(_protocol);
    _hal._smartServoCmdRawInstr.write(RAW_RD8B);
    _hal._smartServoCmdRawAddr.write(addr);

    _hal._smartServoCmdValid.write(1);

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP);
        //tDebug( LOG )
        //    << "read wait cmd ack "
        //    << cmd_id
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }

    return   _hal._smartServoCmdRawData.read<uint8_t>();

}
uint16_t SmartServo::setRawRead16(uint8_t addr)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "raw read16 command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_RAW);
    _hal._smartServoCmdDevId.write(_busId);

    _hal._smartServoCmdRawOnHold.write(0);
    _hal._smartServoCmdRawProtocol.write(_protocol);
    _hal._smartServoCmdRawInstr.write(RAW_RD16B);
    _hal._smartServoCmdRawAddr.write(addr);

    _hal._smartServoCmdValid.write(1);

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP);
        //tDebug( LOG )
        //    << "read wait cmd ack "
        //    << cmd_id
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
    return  _hal._smartServoCmdRawData.read<uint16_t>();
}
void SmartServo::setRawAction(void)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    tInfo( LOG ) << "raw action command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_RAW);
    _hal._smartServoCmdDevId.write(_busId);

    _hal._smartServoCmdRawOnHold.write(0);
    _hal._smartServoCmdRawProtocol.write(_protocol);
    _hal._smartServoCmdRawInstr.write(RAW_ACTION);

    _hal._smartServoCmdValid.write(1);

    tInfo( LOG ) << "exec command";

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}

void SmartServo::changeId(uint8_t newId)
{
    uint8_t addr_enable;
    uint8_t addr_id;

    if (_protocol == SMART_SERVO_FEETECH)
    {
        addr_enable = FEETECH_REGS_TORQUE_SWITCH;
        addr_id = FEETECH_REGS_ID;
    } else {
        addr_enable = DYNAMIXEL_REGS_TORQUE_ENABLE;
        addr_id = DYNAMIXEL_REGS_ID;
    }
    tInfo( LOG ) << "current id " << _busId << " : " << setRawRead8(addr_id);

    setRawWrite8(addr_enable,false);
    setRawWrite8(addr_id,newId);
    _busId = newId;
    registerDevice();
    setRawWrite8(addr_enable,true);

    tInfo( LOG ) << "id changed to " << _busId << "check : " << setRawRead8(addr_id);

}


void SmartServo::setPosition(bool onHold, bool waitPosition, uint16_t pos)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "set pos " << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_SET_POS);
    _hal._smartServoCmdDevId.write(_devId);
    _hal._smartServoCmdSetPosOnHold.write(onHold);
    _hal._smartServoCmdSetPosWaitPosition.write(waitPosition);
    _hal._smartServoCmdSetPosPosition.write(pos);

    _hal._smartServoCmdValid.write(1);
    //tInfo( LOG ) << "exec command";

    while ( _hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write pos wait cmd ack "
        //    <<  _hal._smartServoCmdAck.read<uint8_t>();
    }

}

void SmartServo::setPositionAndSpeed(bool onHold, bool waitPosition, uint16_t pos, uint16_t speed)
{

    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "write pos and speed " << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_SET_POS_AND_SPEED);
    _hal._smartServoCmdDevId.write(_devId);
    _hal._smartServoCmdSetPosAndSpeedOnHold.write(onHold);
    _hal._smartServoCmdSetPosAndSpeedWaitPosition.write(waitPosition);
    _hal._smartServoCmdSetPosAndSpeedPosition.write(pos);
    _hal._smartServoCmdSetPosAndSpeedSpeed.write(speed);

    _hal._smartServoCmdValid.write(1);
    //tInfo( LOG ) << "exec command";

    while ( _hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write pos wait cmd ack "
        //    <<  _hal._smartServoCmdAck.read<uint8_t>();
    }

}
void SmartServo::setAction(bool waitPosition)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "set action command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_SET_ACTION);
    _hal._smartServoCmdDevId.write(0); // not used
    _hal._smartServoCmdSetActionWaitPosition.write(waitPosition);


    _hal._smartServoCmdValid.write(1);
    //tInfo( LOG ) << "exec command";

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write pos wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}
void SmartServo::checkStatus()
{
    std::unique_lock<std::mutex> lck(_cmdMutex);
    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    //tInfo( LOG ) << "check status command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_GET_STATUS);
    _hal._smartServoCmdDevId.write(_devId);

    _hal._smartServoCmdValid.write(1);

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP);
        //tDebug( LOG )
        //    << "read wait cmd ack "
        //    << cmd_id
        //    << _hal._smartServoCmdAck.read<uint8_t>();

    }
    _current_pos        = _hal._smartServoCmdGetStatusPosition.read<uint16_t>();
    _current_load       = _hal._smartServoCmdGetStatusLoad.read<uint16_t>();;
    _current_voltage    = _hal._smartServoCmdGetStatusVoltage.read<uint8_t>();;
    _current_temp       = _hal._smartServoCmdGetStatusTemp.read<uint8_t>();;

}

uint16_t SmartServo::getPosition(bool Update)
{
    //if (Update)
    //    checkStatus();
    uint8_t addr_current_pos;

    if (_protocol == SMART_SERVO_FEETECH)
    {
        addr_current_pos = FEETECH_REGS_CURRENT_POS_H;
    } else {
        addr_current_pos = DYNAMIXEL_REGS_PRESENT_POSITION_L;
    }
    _current_pos = setRawRead16(addr_current_pos);
    return _current_pos;
}
uint16_t SmartServo::getLoad(bool Update)
{
    if (Update)
        checkStatus();
    return _current_load;
}
uint8_t SmartServo::getVoltage(bool Update)
{
    if (Update)
        checkStatus();
    return _current_voltage;
}
uint8_t SmartServo::getTemp(bool Update)
{
    if (Update)
        checkStatus();
    return _current_temp;
}

void SmartServo::setEnable(bool onHold, bool enable)
{
    std::unique_lock<std::mutex> lck(_cmdMutex);

    uint8_t cmd_id = _hal._smartServoCmdAck.read< uint8_t >();

    _hal._smartServoCmdValid.write(0);
    tInfo( LOG ) << "set enable command" << _devId;
    _hal._smartServoCmdId.write(cmd_id++);

    _hal._smartServoCmdType.write(CMD_TYPE_SET_ENABLE);
    _hal._smartServoCmdDevId.write(_devId); // not used
    _hal._smartServoCmdSetEnableOnHold.write(onHold);
    _hal._smartServoCmdSetEnableEnable.write(enable);

    _hal._smartServoCmdValid.write(1);
    tInfo( LOG ) << "exec command";

    while (_hal._smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        //tDebug( LOG )
        //    << "write pos wait cmd ack "
        //    << _hal._smartServoCmdAck.read<uint8_t>();
    }
}

bool SmartServo::moving()
{
    uint8_t moving_bits = _hal._smartServoCmdValid.read<uint8_t>();

    uint8_t ret;
    //tInfo( LOG ) << _hal._smartServoCmdValid.read<uint8_t>();

    ret = (moving_bits >> _devId) & 0x01;
    return ret;
}


const QString& SmartServo::name() const
{
    return _name;
}

uint16_t SmartServo::read()
{
    //return _servo->read< uint16_t >();
}

void SmartServo::write( uint16_t read )
{
    //_servo->write( read );
}

void SmartServo::enable()
{
    setEnable(false,true);
}

void SmartServo::disable()
{
    setEnable(false,false);
}

bool SmartServo::isAttached() const
{
    return true;
}

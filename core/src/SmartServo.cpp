// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>
#include <exception>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SmartServo.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;


class SmartServoBusError: public std::exception
{
    std::string m_msg;
    public:
    SmartServoBusError(const std::string& from, const std::string& info)
        : m_msg(std::string("SmartServo Bus Error from ") + from + std::string(", info ") + info)
    {}
    virtual const char* what() const throw()
    {
        return m_msg.c_str();
    }
};

class SmartServoCmdTimeout: public std::exception
{
    std::string m_msg;
    public:
    SmartServoCmdTimeout(const std::string& from, const std::string& info)
        : m_msg(std::string("SmartServo Command Timeout from ") + from + std::string(", info ") + info)
    {}
    virtual const char* what() const throw()
    {
        return m_msg.c_str();
    }
};
/*
class SmartServoCmdTimeout: public std::exception
{
  virtual const char* what() const throw()
  {
    return "SmartServo Command Timeout Error";
  }
} SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
*/

class SmartServoOutOfID: public std::exception
{
  virtual const char* what() const throw()
  {
    return "SmartServo Too many Servos have been instantiated";
  }
} _smartServoOutOfID;

SmartServoStaticData SmartServo::_staticData;
std::mutex SmartServo::_cmdMutex;
QMutex SmartServo::_qcmdMutex;

#define CMD_ACK_SLEEP 2
#define CMD_TIMEOUT_MS 2000
//#define USE_QMUTEX

SmartServo::SmartServo( const QString& name )
    : _name( name )
    , _attached( false )
{
}

SmartServo::~SmartServo()
{
    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_staticData._mutex);
    #else
    QMutexLocker locker( & _staticData._qmutex );
    #endif
    tDebug( LOG ) << "SmartServo [" << _name << "] destructor engaged";

    if (_attached)
    {

        // we disable the servo, but in the case the servo is no more working / connected
        // we then need to catch any underlying exception
        try {
            disable();
        } catch (...)
        {

        }
        // we give back the internal _devId so it could be used again
        _staticData._qId.push(_devId);
        _staticData._deviceList[_devId] = nullptr;
        tDebug( LOG ) << "SmartServo [" << _name << "] device id" << _devId << "is pushed back";
    }

}


bool SmartServo::attach(
        const Hal::Ptr& hal,
        uint8_t protocol,
        uint8_t busId) // id on the hw bus
{
    bool ret = false;
    _protocol = protocol;
    _busId = busId;
    _hal = hal;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_staticData._mutex);
    #else
    QMutexLocker locker( & _staticData._qmutex );
    #endif

    if (!_staticData._qId.empty())
    {
        _devId = _staticData._qId.front();
        _staticData._deviceList[_devId] = this;

        registerDevice();
        ret = true;
        _attached = true;
        // we pop only if we succeed to register the device
        _staticData._qId.pop();

        tDebug( LOG ) << "SmartServo [" << _name << "] attached to devId" << _devId << "protocol " << _protocol << "busId " << _busId;
    } else {
        tFatal( LOG ) << "SmartServo [" << _name << "] cannot be attached (out of internal id)";
        throw _smartServoOutOfID;
    }


    return ret;
}

void SmartServo::registerDevice()
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] Registering device devId" << _devId << "protocol" << _protocol << "busId" << _busId;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    // REG DEVICE
    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_REGISTER_DEV);
    _hal->_smartServoCmdDevId.write(_devId);
    _hal->_smartServoCmdRegisterProtocol.write(_protocol); // dyna
    _hal->_smartServoCmdRegisterBusId.write(_busId);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}


void SmartServo::setRawWrite8(uint8_t addr, uint8_t data)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] RAW Write 8bit to addr" << addr << "data" << data;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_RAW);
    _hal->_smartServoCmdDevId.write(_busId);

    _hal->_smartServoCmdRawOnHold.write(0);
    _hal->_smartServoCmdRawProtocol.write(_protocol);
    _hal->_smartServoCmdRawInstr.write(RAW_WR8B);
    _hal->_smartServoCmdRawAddr.write(addr);
    _hal->_smartServoCmdRawData.write(data);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}
void SmartServo::setRawWrite16(uint8_t addr, uint16_t data)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] RAW Write 16bit to addr" << addr << "data" << data;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_RAW);
    _hal->_smartServoCmdDevId.write(_busId);

    _hal->_smartServoCmdRawOnHold.write(0);
    _hal->_smartServoCmdRawProtocol.write(_protocol);
    _hal->_smartServoCmdRawInstr.write(RAW_WR16B);
    _hal->_smartServoCmdRawAddr.write(addr);
    _hal->_smartServoCmdRawData.write(data);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}
uint8_t SmartServo::getRawRead8(uint8_t addr)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] RAW Read 8bit to addr" << addr;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_RAW);
    _hal->_smartServoCmdDevId.write(_busId);

    _hal->_smartServoCmdRawOnHold.write(0);
    _hal->_smartServoCmdRawProtocol.write(_protocol);
    _hal->_smartServoCmdRawInstr.write(RAW_RD8B);
    _hal->_smartServoCmdRawAddr.write(addr);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));

    return   _hal->_smartServoCmdRawData.read<uint8_t>();

}
uint16_t SmartServo::getRawRead16(uint8_t addr)
{
    //tDebug( LOG ) <<  "SmartServo [" << _name << "] RAW Read 16bit to addr" << addr;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif


    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_RAW);
    _hal->_smartServoCmdDevId.write(_busId);

    _hal->_smartServoCmdRawOnHold.write(0);
    _hal->_smartServoCmdRawProtocol.write(_protocol);
    _hal->_smartServoCmdRawInstr.write(RAW_RD16B);
    _hal->_smartServoCmdRawAddr.write(addr);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));

    return  _hal->_smartServoCmdRawData.read<uint16_t>();
}
void SmartServo::setRawAction(void)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] set RAW Action";

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif


    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_RAW);
    _hal->_smartServoCmdDevId.write(_busId);

    _hal->_smartServoCmdRawOnHold.write(0);
    _hal->_smartServoCmdRawProtocol.write(_protocol);
    _hal->_smartServoCmdRawInstr.write(RAW_ACTION);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}

void SmartServo::changeId(uint8_t newId)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] Changing busId from" << _busId << "to" << newId;

    uint8_t addr_enable;
    uint8_t addr_id;

    if (_protocol == SMART_SERVO_FEETECH)
    {
        addr_enable = FEETECH_REGS_TORQUE_SWITCH;
        addr_id = FEETECH_REGS_ID;

        // disable EEPROM locks
        setRawWrite8(FEETECH_REGS_LOCK_SIGN,0);

    } else {
        addr_enable = DYNAMIXEL_REGS_TORQUE_ENABLE;
        addr_id = DYNAMIXEL_REGS_ID;
    }

    setRawWrite8(addr_enable,false);
    setRawWrite8(addr_id,newId);
    _busId = newId;
    registerDevice();
    setRawWrite8(addr_enable,true);

}


void SmartServo::setPosition(bool onHold, bool waitPosition, uint16_t pos)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] New Pos" << pos << "onHold" << onHold << "waitPosition" << waitPosition;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_SET_POS);
    _hal->_smartServoCmdDevId.write(_devId);
    _hal->_smartServoCmdSetPosOnHold.write(onHold);
    _hal->_smartServoCmdSetPosWaitPosition.write(waitPosition);
    _hal->_smartServoCmdSetPosPosition.write(pos);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}

void SmartServo::setPositionAndSpeed(bool onHold, bool waitPosition, uint16_t pos, uint16_t speed)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] New Pos" << pos << "and Speed" << speed << "onHold" << onHold << "waitPosition" << waitPosition;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_SET_POS_AND_SPEED);
    _hal->_smartServoCmdDevId.write(_devId);
    _hal->_smartServoCmdSetPosAndSpeedOnHold.write(onHold);
    _hal->_smartServoCmdSetPosAndSpeedWaitPosition.write(waitPosition);
    _hal->_smartServoCmdSetPosAndSpeedPosition.write(pos);
    _hal->_smartServoCmdSetPosAndSpeedSpeed.write(speed);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}
void SmartServo::setAction(bool waitPosition)
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] Set Action / waitPosition" << waitPosition;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif


    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_SET_ACTION);
    _hal->_smartServoCmdDevId.write(0); // not used
    _hal->_smartServoCmdSetActionWaitPosition.write(waitPosition);


    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));
}
void SmartServo::checkStatus()
{
    tDebug( LOG ) <<  "SmartServo [" << _name << "] Check Status";

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif


    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_GET_STATUS);
    _hal->_smartServoCmdDevId.write(_devId);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));

    _current_pos        = _hal->_smartServoCmdGetStatusPosition.read<uint16_t>();
    _current_load       = _hal->_smartServoCmdGetStatusLoad.read<uint16_t>();
    _current_voltage    = _hal->_smartServoCmdGetStatusVoltage.read<uint8_t>();
    _current_temp       = _hal->_smartServoCmdGetStatusTemp.read<uint8_t>();

}

uint16_t SmartServo::getPosition(bool Update)
{
    Q_UNUSED( Update );

    //if (Update)
    //    checkStatus();
    uint8_t addr_current_pos;

    if (_protocol == SMART_SERVO_FEETECH)
    {
        addr_current_pos = FEETECH_REGS_CURRENT_POS_H;
    } else {
        addr_current_pos = DYNAMIXEL_REGS_PRESENT_POSITION_L;
    }
    _current_pos = getRawRead16(addr_current_pos);
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
    tDebug( LOG ) <<  "SmartServo [" << _name << "] set Enable \\ onHold" << onHold << "enable" << enable;

    #ifndef USE_QMUTEX
    std::unique_lock<std::mutex> lck(_cmdMutex);
    #else
    QMutexLocker locker( & _qcmdMutex );
    #endif

    uint8_t cmd_id = _hal->_smartServoCmdAck.read< uint8_t >();

    _hal->_smartServoCmdValid.write(0);
    _hal->_smartServoCmdId.write(cmd_id++);

    _hal->_smartServoCmdType.write(CMD_TYPE_SET_ENABLE);
    _hal->_smartServoCmdDevId.write(_devId); // not used
    _hal->_smartServoCmdSetEnableOnHold.write(onHold);
    _hal->_smartServoCmdSetEnableEnable.write(enable);

    _hal->_smartServoCmdValid.write(1);

    uint32_t timeout = 0;
    while (_hal->_smartServoCmdAck.read<uint8_t>() != cmd_id)
    {
        QThread::msleep( CMD_ACK_SLEEP );
        timeout += CMD_ACK_SLEEP;
        if (timeout >= CMD_TIMEOUT_MS)
            throw SmartServoCmdTimeout(std::string{__FUNCTION__}+":"+std::to_string(__LINE__),_name.toStdString() + "|devId:" + std::to_string(_devId) + "|busId:" + std::to_string(_busId));;
    }
    if (_hal->_smartServoCmdError.read<uint8_t>())
        throw SmartServoBusError(std::string{__FUNCTION__},std::to_string(_devId)+":"+std::to_string(enable));
}

bool SmartServo::moving()
{
    uint8_t moving_bits = _hal->_smartServoCmdValid.read<uint8_t>();
    uint8_t ret;

    ret = (moving_bits >> _devId) & 0x01;
    return ret;
}


const QString& SmartServo::name() const
{
    return _name;
}

void SmartServo::enable()
{
    setEnable(false,true);
    tInfo( LOG ) <<  "SmartServo [" << _name << "] Enabled";

}

void SmartServo::disable()
{
    setEnable(false,false);
    tInfo( LOG ) <<  "SmartServo [" << _name << "] Disabled";
}

bool SmartServo::isAttached() const
{
    return _attached;
}

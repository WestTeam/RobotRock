// Copyright (c) 2019 All Rights Reserved WestBot

#include <QByteArray>
#include <QString>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Vl6180x.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.VL6180X" )
}

Vl6180x::Vl6180x( const QString& tty, QObject* parent )
    : QObject( parent )
    , _distance( 0 )
{
    open( tty );

    connect(
        & _serial,
        & QSerialPort::readyRead,
        this,
        & Vl6180x::readData );
}

Vl6180x::~Vl6180x()
{
    _serial.close();
}

uint32_t Vl6180x::distance() const
{
    return _distance;
}

//
// Private methods
//
void Vl6180x::open( const QString& tty )
{
    _serial.setPortName( tty );
    _serial.setBaudRate( QSerialPort::Baud115200 );
    _serial.setDataBits( QSerialPort::Data8 );
    _serial.setParity( QSerialPort::NoParity );
    _serial.setStopBits( QSerialPort::OneStop );
    _serial.setFlowControl( QSerialPort::NoFlowControl );

    if( _serial.open( QIODevice::ReadOnly ) )
    {
        tDebug( LOG ) << "Vl6180x TTY opened";
    }
    else
    {
        tFatal( LOG ) << "Vl6180x TTY failed to open";
    }
}

void Vl6180x::readData()
{
    const QByteArray data = _serial.readAll();

    if( data[ 0 ] = 0x01 && data[ Ã2 ] == 0xA5 )
    {
        // Valid trame found
        _distance = data[ 1 ];
    }
}

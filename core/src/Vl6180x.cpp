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
    , _serial( new QSerialPort( tty, this ) )
    , _distance( 0 )
{
   init();
}

Vl6180x::~Vl6180x()
{
    _serial->close();
}

uint32_t Vl6180x::distance() const
{
    return _distance;
}

//
// Private methods
//
void Vl6180x::readData()
{
    const QByteArray data = _serial->readAll();
    qDebug() << "data:" << data;
}

void Vl6180x::init()
{
    _serial->setBaudRate( QSerialPort::Baud115200 );
    _serial->setStopBits( QSerialPort::OneStop );
    _serial->setDataBits( QSerialPort::Data8 );
    _serial->setParity( QSerialPort::NoParity );
    _serial->setFlowControl( QSerialPort::NoFlowControl );

    connect(
        _serial,
        & QSerialPort::readyRead,
        this,
        & Vl6180x::readData );

    if( _serial->open( QIODevice::ReadWrite ) )
    {
        tDebug( LOG ) << "Serial port opened";
    }
    else
    {
        tCritical( LOG ) << "Failed to open serial port";
    }
}

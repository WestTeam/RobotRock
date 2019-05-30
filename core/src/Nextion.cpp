// Copyright (c) 2019 All Rights Reserved WestBot

#include <QMutexLocker>
#include <QString>
#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Nextion.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Nextion" )
}

Nextion::Nextion( const QString& tty )
    : _lock( new QMutex( QMutex::NonRecursive ) )
{
    open( tty );
}

Nextion::~Nextion()
{
    _serial.close();
}

void Nextion::send( QByteArray label, QByteArray val )
{
    QByteArray str = label + ".txt=\"" + val + "\"";
    write( str );
}

void Nextion::setColor( QByteArray label, QByteArray color )
{
    QByteArray str = label + ".bco=" + color;
    write( str );
}

//
// Private methods
//
void Nextion::open( const QString& tty )
{
    _serial.setPortName( tty );
    _serial.setBaudRate( QSerialPort::Baud9600 );
    _serial.setDataBits( QSerialPort::Data8 );
    _serial.setParity( QSerialPort::NoParity );
    _serial.setStopBits( QSerialPort::OneStop );
    _serial.setFlowControl( QSerialPort::NoFlowControl );

    if( _serial.open( QIODevice::ReadWrite ) )
    {
        tDebug( LOG ) << "Nextion TTY opened";
    }
    else
    {
        tFatal( LOG ) << "Nextion TTY failed to open";
    }
}

void Nextion::write( QByteArray data )
{
    QByteArray ack;
    ack.resize( 3 );
    ack[ 0 ] = 0xFF;
    ack[ 1 ] = 0xFF;
    ack[ 2 ] = 0xFF;

    //QMutexLocker locker( _lock );

    _serial.write( data );
    _serial.write( ack );

    _serial.flush();

    QThread::msleep( 10 );
}

// Copyright (c) 2019 All Rights Reserved WestBot

#include <QString>
#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Experiment.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Experiment" )
}

Experiment::Experiment( const QString& tty, QObject* parent )
    : QObject( parent )
    , _serial( new QSerialPort( tty, this ) )
{
    init();
}

Experiment::~Experiment()
{
    _serial->close();
}

void Experiment::start()
{
    const QByteArray str( "s" );
    write( str );
}

void Experiment::setColorYellow()
{
    const QByteArray str( "1" );
    write( str );
}

void Experiment::setColorPurple()
{
    const QByteArray str( "0" );
    write( str );
}

//
// Private methods
//
void Experiment::init()
{
    _serial->setBaudRate( QSerialPort::Baud9600 );
    _serial->setStopBits( QSerialPort::OneStop );
    _serial->setDataBits( QSerialPort::Data8 );
    _serial->setParity( QSerialPort::NoParity );
    _serial->setFlowControl( QSerialPort::NoFlowControl );

    if( _serial->open( QIODevice::ReadWrite ) )
    {
        tDebug( LOG ) << "Experiment TTY opened";
    }
    else
    {
        tCritical( LOG ) << "Failed to open Experiment TTY";
    }
}

void Experiment::write( QByteArray data )
{
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
    _serial->write( data );
    _serial->flush();
}

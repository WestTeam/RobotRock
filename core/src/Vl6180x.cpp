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

    uint16_t protocolCrc( uint8_t* msg, uint16_t size )
    {
        uint16_t crc = 0;

        while( --size > 4 )
        {
            crc += msg[ size ];
        }

        return crc;
    }
}

typedef struct
{
    uint8_t     fanion;
    uint16_t    size;
    uint16_t    crc; // sum data + id
    uint16_t    id;
} __attribute__( ( packed ) ) ProtocolHeader;

typedef struct
{
    ProtocolHeader header;
    uint8_t dist;
    uint8_t status;
} __attribute__( ( packed ) ) Trame;

Trame trame;

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

uint8_t Vl6180x::distance() const
{
    return _distance;
}

//
// Private methods
//
void Vl6180x::readData()
{
    while( _serial->bytesAvailable() >= 9 )
    {
        _serial->read( ( char* ) & trame.header.fanion, 1 );
        if( trame.header.fanion == 0xA5 )
        {
            _serial->read( ( char* ) & trame.header.size, 8 );

            const uint16_t crc = ::protocolCrc(
                ( uint8_t* ) & trame,
                sizeof( Trame ) );

            if( crc != trame.header.crc )
            {
                tWarning( LOG ) << "Trame not valid: CRC error";
                return;
            }

            if( trame.status == 0 )
            {
                _distance = trame.dist;
            }
            else
            {
                tWarning( LOG ) << "Sensor status out of range: dropping data";
            }
        }
    }
}

void Vl6180x::init()
{
    _serial->setBaudRate( QSerialPort::Baud9600 );
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

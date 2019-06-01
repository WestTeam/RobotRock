// Copyright (c) 2019 All Rights Reserved WestBot

#include <QByteArray>
#include <QString>
#include <QDate>


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

Vl6180x::Vl6180x( const QString& tty )
{
    _tty = tty;
   start();
}

Vl6180x::~Vl6180x()
{
    _serial->close();
}

double Vl6180x::distance( int sensorId ) const
{
    if (sensorId >= VL6180X_MAX_SENSOR_COUNT)
        return 0.0;

    return static_cast< double >( _distance[ sensorId ] );
}


bool Vl6180x::status( int sensorId )
{
    if (sensorId >= VL6180X_MAX_SENSOR_COUNT)
        return false;

    uint64_t sensorDataTime = _ts[sensorId];
    uint64_t now = QDateTime::currentMSecsSinceEpoch();

    uint64_t diff = now-sensorDataTime;

    if (diff < 1000 && (_status[sensorId] == 0 || _status[sensorId] > 5))
    {
        return true;
    }

    return false;
}

uint32_t* Vl6180x::distancePointer( int sensorId )
{
    if (sensorId >= VL6180X_MAX_SENSOR_COUNT)
        return nullptr;

    return &_distance[sensorId];
}

uint64_t Vl6180x::samplingPeriod( int sensorId )
{
    return _samplingPeriod[sensorId];
}




//
// Private methods
//
void Vl6180x::readData()
{
    _serial->waitForReadyRead(1);

    /*
    if (_serial->bytesAvailable() >= 9)
    {
        char buffer[10];
        _serial->read( ( char* ) buffer, 9 );
    }*/

    while( _serial->bytesAvailable() >= 9 )
//    if (0)
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
                tWarning( LOG ) << _tty << "Trame not valid: CRC error" << crc << trame.header.crc;
                return;
            } else {
                //tInfo( LOG ) << _tty << "Trame OK";
            }
            /*else {
                tInfo( LOG ) << _tty << "OK";

            }*/

            if (trame.header.id < VL6180X_MAX_SENSOR_COUNT)
            {
                _status[ trame.header.id ] = trame.status;

                if (trame.status == 0)
                    _distance[ trame.header.id ] = trame.dist;
                else
                    _distance[ trame.header.id ] = 255;

                uint64_t now = QDateTime::currentMSecsSinceEpoch();

                if (_ts[ trame.header.id ] != 0)
                    _samplingPeriod [trame.header.id] = now-_ts[ trame.header.id ];

                _ts[ trame.header.id ] = now;


                if (!(trame.status == 0 || trame.status >= 5))
                {
                    tWarning( LOG ) << _tty << "Sensor status hardware error: dropping data" << trame.status << trame.header.id;
                }
            } else {
                tWarning( LOG ) << _tty << "Sensor Id cannot be hanlded: dropping data" << trame.status << trame.header.id;
            }
        }
    }
}

void Vl6180x::init()
{

    for (int i = 0; i < VL6180X_MAX_SENSOR_COUNT; i++)
    {
        _distance[i] = 255;
        _status[i] = 1;
        _ts[i] = 0;
        _samplingPeriod[i] = 0;
    }

    _serial->setBaudRate( QSerialPort::Baud115200 );
    _serial->setStopBits( QSerialPort::OneStop );
    _serial->setDataBits( QSerialPort::Data8 );
    _serial->setParity( QSerialPort::NoParity );
    _serial->setFlowControl( QSerialPort::NoFlowControl );

    /*connect(
        _serial,
        & QSerialPort::readyRead,
        this,
        & Vl6180x::readData );


    */

    if( _serial->open( QIODevice::ReadWrite ) )
    {
        tDebug( LOG ) << "VL6180X TTY opened";
    }
    else
    {
        tCritical( LOG ) << "Failed to open VL6180X TTY";
    }
}

void Vl6180x::run()
{
    _serial = new QSerialPort( _tty );
    _serial->moveToThread(this);
    init();

    while (1)
    {
        readData();
    }

    //while( 1 );
}

// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_NEXTION_HPP_
#define WESTBOT_ROBOTROCK_NEXTION_HPP_

#include <QByteArray>
#include <QSerialPort>
#include <QMutex>

class QString;

namespace WestBot {
namespace RobotRock {

class Nextion
{
public:
    Nextion( const QString& tty = "/dev/ttyAL0" );
    ~Nextion();

    void send( QByteArray label, QByteArray val );
    void setColor( QByteArray label, QByteArray color );

private:
    void open( const QString& tty );
    void write( QByteArray data );

private:
    QSerialPort _serial;
    QMutex* _lock;
};

}
}

#endif // WESTBOT_ROBOTROCK_NEXTION_HPP_

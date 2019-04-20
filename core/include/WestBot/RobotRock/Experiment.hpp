// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_EXPERIMENT_HPP_
#define WESTBOT_ROBOTROCK_EXPERIMENT_HPP_

#include <QByteArray>
#include <QObject>
#include <QSerialPort>

class QString;

#include "Common.hpp"
#include "Hal.hpp"

namespace WestBot {
namespace RobotRock {

class Experiment : public QObject
{
public:
    Experiment( const QString& tty = "/dev/ttyAL9", QObject* parent = nullptr );
    ~Experiment();

    void start();
    void setColorYellow();
    void setColorPurple();

private:
    void init();
    void write( QByteArray data );

private:
    QSerialPort* _serial;
};

}
}

#endif // WESTBOT_ROBOTROCK_EXPERIMENT_HPP_

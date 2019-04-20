// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_VL6180X_HPP_
#define WESTBOT_ROBOTROCK_VL6180X_HPP_

#include <QObject>
#include <QSerialPort>

class QString;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Vl6180x class process incoming data from a serial link.
 */
class Vl6180x : public QObject
{
public:
    Vl6180x( const QString& tty = "/dev/ttyAL8", QObject* parent = nullptr );

    ~Vl6180x();

    /*!
     * \brief Return the last distance from the VL6180X sensor.
     *
     * \return Distance in mm.
     */
    uint8_t distance() const;

private slots:
    void readData();

private:
    void init();

private:
    QSerialPort* _serial;
    uint32_t _distance;
};

}
}

#endif // WESTBOT_ROBOTROCK_VL6180X_HPP_


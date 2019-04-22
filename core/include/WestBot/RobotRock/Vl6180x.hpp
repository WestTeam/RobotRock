// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_VL6180X_HPP_
#define WESTBOT_ROBOTROCK_VL6180X_HPP_

#include <QSerialPort>
#include <QThread>

class QString;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Vl6180x class process incoming data from a serial link.
 */
class Vl6180x : public QThread
{
public:
    Vl6180x( const QString& tty = "/dev/ttyAL8" );

    ~Vl6180x();

    /*!
     * \brief Return the last distance from the VL6180X sensor.
     *
     * \return Distance in mm.
     */
    uint8_t distance() const;

private:
    void run() override;

    void init();
    void readData();

private:
    QSerialPort* _serial;
    uint32_t _distance;
};

}
}

#endif // WESTBOT_ROBOTROCK_VL6180X_HPP_


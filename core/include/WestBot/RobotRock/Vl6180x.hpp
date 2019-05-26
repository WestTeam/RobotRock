// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_VL6180X_HPP_
#define WESTBOT_ROBOTROCK_VL6180X_HPP_

#include <memory>

#include <QSerialPort>
#include <QThread>

class QString;

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Vl6180x class process incoming data from a serial link.
 */

#define VL6180X_MAX_SENSOR_COUNT 4

class Vl6180x : public QThread
{
public:
    //using Ptr = std::shared_ptr< Vl6180x >;

    Vl6180x( const QString& tty = "/dev/ttyAL8" );

    ~Vl6180x() override;

    /*!
     * \brief Return the last distance from the VL6180X sensor.
     *
     * \return Distance in mm.
     */
    double distance( int sensorId ) const;

    /*!
     * \brief Return the status of sensor
     *
     * \return boolean.
     */
    bool status( int sensorId );

    /*!
     * \brief Return the distance pointer associated to the sensor
     *
     * \return pointer.
     */
    uint32_t* distancePointer( int sensorId );

    /*!
     * \brief Return the latest sampling Period
     *
     * \return period in Ms.
     */
    uint64_t samplingPeriod( int sensorId );


private:
    void run() override;

    void init();
    void readData();

private:
    QSerialPort* _serial;
    uint32_t _distance[ VL6180X_MAX_SENSOR_COUNT ];
    uint8_t  _status[ VL6180X_MAX_SENSOR_COUNT ];
    uint64_t _ts[ VL6180X_MAX_SENSOR_COUNT ];//std::atomic<uint64_t> _ts[ VL6180X_MAX_SENSOR_COUNT ];
    uint64_t _samplingPeriod[ VL6180X_MAX_SENSOR_COUNT ];//std::atomic<uint64_t> _samplingPeriod[ VL6180X_MAX_SENSOR_COUNT ];

};

}
}

#endif // WESTBOT_ROBOTROCK_VL6180X_HPP_


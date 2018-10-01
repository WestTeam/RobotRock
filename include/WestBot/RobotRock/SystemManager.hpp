// COPYRIGHT (C) 2018 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

#include <QObject>

#include "Common.hpp"
#include "Hal.hpp"
#include "Recalage.hpp"
#include "TrajectoryManager.hpp"

class QState;
class QString;

namespace WestBot {
namespace RobotRock {

class SystemManager : public QObject
{
    Q_OBJECT

public:
    enum class SystemMode
    {
        Free, //No PID -> register = 0x00
        DistanceOnly, //PIDD -> register = 0x01
        AngleOnly, //PIDA -> register = 0x02
        Full, //All PID -> register = 0x03
        Unused // 0xFF : Unused for now
    };

    SystemManager( Hal& hal, QObject* parent = nullptr );

    ~SystemManager() override;

    bool init();

    void start();
    void stop();
    void reset();

    void setMode( SystemMode mode );
    SystemMode mode() const;

private:
    Hal _hal;
    Recalage _recalage;
    TrajectoryManager _trajectoryManager;
    SystemMode _systemMode;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

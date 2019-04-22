// COPYRIGHT (C) 2018-2019 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

#include <QObject>
#include <QTimer>

#include "Common.hpp"
#include "Experiment.hpp"
#include "GameThread.hpp"
#include "Input.hpp"
#include "Lidar.hpp"
#include "Monitoring.hpp"
#include "Output.hpp"
#include "Recalage.hpp"
#include "StrategyManager.hpp"
#include "TrajectoryManager.hpp"
#include "Vl6180x.hpp"

class QState;
class QString;

namespace WestBot {
namespace RobotRock {

class SystemManager : public QObject
{
public:
    enum class SystemMode
    {
        Free, // No PID -> register = 0x00
        DistanceOnly, // PIDD -> register = 0x01
        AngleOnly, // PIDA -> register = 0x02
        Full, // All PID -> register = 0x03
        Unused // 0xFF : Unused for now
    };

    SystemManager( const Hal::Ptr& hal, QObject* parent = nullptr );

    ~SystemManager() override;

    bool init();

    void start();
    void stop();
    void reset();

    void setMode( SystemMode mode );
    SystemMode mode() const;

    bool isSafe() const;

private:
    void initRecalage();
    void blinkColorLed();
    void robotAlive();
    void displayColor( const DigitalValue& value );

private:
    Hal::Ptr _hal;
    QTimer _gameTimer;
    QTimer _aliveTimer;
    Input::Ptr _startButton;
    Input::Ptr _colorButton;
    Input::Ptr _hardstopButton;
    Output::Ptr _ledYellow;
    Output::Ptr _ledBlue;
    Color _color;
    SystemMode _systemMode;

    // Base system
    Odometry::Ptr _odometry;
    Recalage::Ptr _recalage;
    LidarRPLidarA2::Ptr _lidar;
    TrajectoryManager::Ptr _trajectoryManager;
    StrategyManager::Ptr _strategyManager;
    Monitoring::Ptr _monitoring;
    GameThread::Ptr _game;
    Vl6180x _distanceSensor;
    Experiment _experiment;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

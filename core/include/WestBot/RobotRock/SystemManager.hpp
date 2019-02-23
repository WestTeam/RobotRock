// COPYRIGHT (C) 2018-2019 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

#include <QObject>
#include <QTimer>

#include "Common.hpp"
#include "GameThread.hpp"
#include "Input.hpp"
#include "Lidar.hpp"
#include "Output.hpp"
#include "Recalage.hpp"
#include "StrategyManager.hpp"
#include "TrajectoryManager.hpp"

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
    Recalage::Ptr _recalage;
    Lidar _lidar;
    TrajectoryManager _trajectoryManager;
    SystemMode _systemMode;
    StrategyManager _strategyManager;
    GameThread::Ptr _game;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

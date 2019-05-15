// COPYRIGHT (C) 2018-2019 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

#include <QObject>
#include <QTimer>

#include "Common.hpp"
#include "Input.hpp"
#include "Output.hpp"

namespace WestBot {
namespace RobotRock {

class SystemManager : public QObject
{
public:
    SystemManager( QObject* parent = nullptr );

    ~SystemManager() override = default;

    virtual bool init() = 0;

    virtual void start() = 0;
    virtual void stop();
    virtual void reset() = 0;

    virtual bool isSafe() const = 0;

protected:
    virtual void blinkColorLed();
    virtual void displayColor( const DigitalValue& value ) = 0;

protected:
    QTimer _gameTimer;

    Input::Ptr _startButton;
    Input::Ptr _colorButton;
    Input::Ptr _hardstopButton;
    Output::Ptr _ledYellow;
    Output::Ptr _ledBlue;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGER_HPP_

// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_
#define WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_

#include <QObject>

namespace WestBot {
namespace RobotRock {

class OpponentDetection : public QObject
{
    Q_OBJECT

public:
    OpponentDetection();
    ~OpponentDetection() override = default;

    // TODO: Remove this just to understand
    void methodFoo();

signals:
    // Signals does not return something. Always void...
    void opponentDetected( double x, double y );
};

}
}

#endif // WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_

# Copyright (c) 2018-2019 All Rights Reserved WestBot

QT += core network serialport
QT -= gui

TEMPLATE = lib
CONFIG += staticlib

INCLUDEPATH += include \
               ../3rdparty/Eigen/

SOURCES += \
    src/Action.cpp \
    src/ArmAction.cpp \
    src/AStarHighLevel.cpp \
    src/Experiment.cpp \
    src/GameThread.cpp \
    src/Hal.cpp \
    src/Input.cpp \
    src/InputHw.cpp \
    src/InputSimu.cpp \
    src/ItemRegister.cpp \
    src/Lidar.cpp \
    src/Memory.cpp \
    src/MemoryManager.cpp \
    src/Monitoring.cpp \
    src/MoveAction.cpp \
    src/Nextion.cpp \
    src/Output.cpp \
    src/OutputHw.cpp \
    src/OutputSimu.cpp \
    src/OpponentDetection.cpp \
    src/Recalage.cpp \
    src/Servo.cpp \
    src/Odometry.cpp \
    src/OdometryHw.cpp \
    src/OdometrySimu.cpp \
    src/SimTcpServer.cpp \
    src/SystemManager.cpp \
    src/SystemManagerHw.cpp \
    src/SystemManagerSimu.cpp \
    src/TrajectoryManagerHw.cpp \
    src/TrajectoryManagerSimu.cpp \
    src/Vl6180x.cpp \
    src/WaitAction.cpp \
    src/SmartServo.cpp \
    src/ArmLowLevel.cpp \
    src/ArmLowLevelSimu.cpp \
    src/ArmHighLevel.cpp \
    src/Pid.cpp \
    src/ArmsManager.cpp \
    src/PuckDetection.cpp \
    src/ArmsManagerAction.cpp \
    src/RecalageAction.cpp

HEADERS += \
    include/Macros.hpp \
    include/WestBot/RobotRock/Action.hpp \
    include/WestBot/RobotRock/ActionList.hpp \
    include/WestBot/RobotRock/ArmAction.hpp \
    include/WestBot/RobotRock/AStarHighLevel.hpp \
    include/WestBot/RobotRock/Common.hpp \
    include/WestBot/RobotRock/Experiment.hpp \
    include/WestBot/RobotRock/GameThread.hpp \
    include/WestBot/RobotRock/Hal.hpp \
    include/WestBot/RobotRock/Input.hpp \
    include/WestBot/RobotRock/InputHw.hpp \
    include/WestBot/RobotRock/InputSimu.hpp \
    include/WestBot/RobotRock/ItemRegister.hpp \
    include/WestBot/RobotRock/Lidar.hpp \
    include/WestBot/RobotRock/Memory.hpp \
    include/WestBot/RobotRock/MemoryManager.hpp \
    include/WestBot/RobotRock/Monitoring.hpp \
    include/WestBot/RobotRock/MoveAction.hpp \
    include/WestBot/RobotRock/Nextion.hpp \
    include/WestBot/RobotRock/OpponentDetection.hpp \
    include/WestBot/RobotRock/Output.hpp \
    include/WestBot/RobotRock/OutputHw.hpp \
    include/WestBot/RobotRock/OutputSimu.hpp \
    include/WestBot/RobotRock/Recalage.hpp \
    include/WestBot/RobotRock/Servo.hpp \
    include/WestBot/RobotRock/Odometry.hpp \
    include/WestBot/RobotRock/OdometryHw.hpp \
    include/WestBot/RobotRock/OdometrySimu.hpp \
    include/WestBot/RobotRock/SimTcpServer.hpp \
    include/WestBot/RobotRock/StrategyManager.hpp \
    include/WestBot/RobotRock/SystemManager.hpp \
    include/WestBot/RobotRock/SystemManagerHw.hpp \
    include/WestBot/RobotRock/SystemManagerSimu.hpp \
    include/WestBot/RobotRock/TrajectoryManager.hpp \
    include/WestBot/RobotRock/TrajectoryManagerHw.hpp \
    include/WestBot/RobotRock/TrajectoryManagerSimu.hpp \
    include/WestBot/RobotRock/Vl6180x.hpp \
    include/WestBot/RobotRock/WaitAction.hpp \
    include/WestBot/RobotRock/SmartServo.hpp \
    include/WestBot/RobotRock/ArmLowLevel.hpp \
    include/WestBot/RobotRock/ArmLowLevelSimu.hpp \
    include/WestBot/RobotRock/ArmHighLevel.hpp \
    include/WestBot/RobotRock/Pid.hpp \
    include/WestBot/RobotRock/ArmsManager.hpp \
    include/WestBot/RobotRock/LidarCircle.hpp \
    include/WestBot/RobotRock/PuckDetection.hpp \
    include/WestBot/RobotRock/ArmsManagerAction.hpp \
    include/WestBot/RobotRock/RecalageAction.hpp

INCLUDEPATH += \
    ../3rdparty/HumanAfterAll/include/ \
    ../3rdparty/rplidar/include/ \
    ../3rdparty/a-star/include/
LIBS += -L../../robot-rock/libs -lHumanAfterAll -lRPLidar -lAStar

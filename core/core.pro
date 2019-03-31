# Copyright (c) 2018 All Rights Reserved WestBot

QT += core network serialport
QT -= gui

TEMPLATE = lib
CONFIG += staticlib

INCLUDEPATH += include \
               ../3rdparty/Eigen/

SOURCES += \
    src/Action.cpp \
    src/GameThread.cpp \
    src/Hal.cpp \
    src/Input.cpp \
    src/ItemRegister.cpp \
    src/Lidar.cpp \
    src/Memory.cpp \
    src/MemoryManager.cpp \
    src/MoveAction.cpp \
    src/Nextion.cpp \
    src/Output.cpp \
    src/Recalage.cpp \
    src/Servo.cpp \
    src/Odometry.cpp \
    src/StrategyManager.cpp \
    src/SystemManager.cpp \
    src/TrajectoryManager.cpp \
    src/WaitAction.cpp \
    src/SmartServo.cpp

HEADERS += \
    include/Macros.hpp \
    include/WestBot/RobotRock/Action.hpp \
    include/WestBot/RobotRock/Common.hpp \
    include/WestBot/RobotRock/GameThread.hpp \
    include/WestBot/RobotRock/Hal.hpp \
    include/WestBot/RobotRock/Input.hpp \
    include/WestBot/RobotRock/ItemRegister.hpp \
    include/WestBot/RobotRock/Lidar.hpp \
    include/WestBot/RobotRock/Memory.hpp \
    include/WestBot/RobotRock/MemoryManager.hpp \
    include/WestBot/RobotRock/MoveAction.hpp \
    include/WestBot/RobotRock/Nextion.hpp \
    include/WestBot/RobotRock/Output.hpp \
    include/WestBot/RobotRock/Recalage.hpp \
    include/WestBot/RobotRock/Servo.hpp \
    include/WestBot/RobotRock/Odometry.hpp \
    include/WestBot/RobotRock/StrategyManager.hpp \
    include/WestBot/RobotRock/SystemManager.hpp \
    include/WestBot/RobotRock/TrajectoryManager.hpp \
    include/WestBot/RobotRock/WaitAction.hpp \
    include/WestBot/RobotRock/SmartServo.hpp

INCLUDEPATH += ../../HumanAfterAll/include/ ../../rplidar/include/
LIBS += -L../../robot-rock/libs -lHumanAfterAll -lRPLidar

# Copyright (c) 2018 All Rights Reserved WestBot

QT += core network serialport
QT -= gui

CONFIG += c++11

TARGET = robotrock
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += include \
               3rdparty/Eigen/

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
    src/Output.cpp \
    src/Recalage.cpp \
    src/Servo.cpp \
    src/StrategyManager.cpp \
    src/SystemManager.cpp \
    src/TrajectoryManager.cpp \
    src/WaitAction.cpp \
    src/main.cpp

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
    include/WestBot/RobotRock/Output.hpp \
    include/WestBot/RobotRock/Recalage.hpp \
    include/WestBot/RobotRock/Servo.hpp \
    include/WestBot/RobotRock/StrategyManager.hpp \
    include/WestBot/RobotRock/SystemManager.hpp \
    include/WestBot/RobotRock/TrajectoryManager.hpp \
    include/WestBot/RobotRock/WaitAction.hpp
	
INCLUDEPATH += ../HumanAfterAll/include/ ../rplidar/include/
LIBS += -L../robot-rock/libs -lHumanAfterAll -lRPLidar

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

target.path = /home/ichiro
INSTALLS += target
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

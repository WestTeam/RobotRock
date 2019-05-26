# Copyright (c) 2019 All Rights Reserved WestBot

QT += core network serialport

TARGET = testEcran
SOURCES = main.cpp

INCLUDEPATH += \
    ../3rdparty/HumanAfterAll/include/ \
    ../3rdparty/rplidar/include/
LIBS += -L../../robot-rock/libs -lHumanAfterAll -lRPLidar

INCLUDEPATH += ../core/include
LIBS += -L../core -lcore

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

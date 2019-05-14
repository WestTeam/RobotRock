# Copyright (c) 2018-2019 All Rights Reserved WestBot

QT += core network serialport
QT -= gui

CONFIG += c++11

TEMPLATE = subdirs
SUBDIRS = core robot-rock test1 \
    testLidar \
    SmartServoTesting testOdometry testEcran \
    testRecalage \
    testRecalageLive \
    testHomologation \
    TestVl6180x \
    testArmLowLevel \
    testArmHighLevel \
    testSimServer \
    testRobotSimu

robot-rock.depedends = core
test1.depends = core
testEcran.depends = core
testHomologation.depends = core
testVl6180x.depends = core

# Copyright (c) 2018-2019 All Rights Reserved WestBot

QT += core network serialport
QT -= gui

CONFIG += c++11

TEMPLATE = subdirs
SUBDIRS = core robot-rock

robot-rock.depedends = core

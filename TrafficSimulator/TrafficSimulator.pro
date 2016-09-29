#-------------------------------------------------
#
# Project created by QtCreator 2016-04-03T01:00:12
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TrafficSimulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    intersection.cpp \
    trafficengine.cpp \
    road.cpp \
    linearroad.cpp \
    direction.cpp \
    vehicle.cpp \
    vehiclepath.cpp \
    vehicleposition.cpp \
    trafficlight.cpp \
    roadobject.cpp \
    vehicleroute.cpp \
    general.cpp \
    settingsdialog.cpp \
    routeselectordialog.cpp \
    helpdialog.cpp

HEADERS  += mainwindow.h \
    intersection.h \
    trafficengine.h \
    constants.h \
    road.h \
    linearroad.h \
    direction.h \
    vehicle.h \
    vehiclepath.h \
    vehicleposition.h \
    trafficlight.h \
    roadobject.h \
    vehicleroute.h \
    general.h \
    settingsdialog.h \
    routeselectordialog.h \
    helpdialog.h

FORMS    += mainwindow.ui \
    settingsdialog.ui \
    routeselectordialog.ui \
    helpdialog.ui

RESOURCES +=

#-------------------------------------------------
#
# Project created by QtCreator 2015-04-01T09:44:42
#
#-------------------------------------------------

QT       += core gui serialport widgets

TARGET = PowerAnalyzer
ICON = icone.ico
RC_ICONS = icone.ico
TEMPLATE = app


SOURCES += main.cpp\
        communication.cpp \
        mainwindow.cpp

HEADERS  += mainwindow.h \
    communication.h

FORMS    += mainwindow.ui

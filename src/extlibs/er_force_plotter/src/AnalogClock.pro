QT       += widgets
TARGET = analogclock
TEMPLATE = lib

CONFIG += c++11

win32 {
	CONFIG += staticlib	
}

DEFINES += ANALOGCLOCK_LIBRARY


unix {
    target.path = /usr/lib
    INSTALLS += target
}

HEADERS += \
    analogclockl_global.h \
    analogclock.h

SOURCES += \
    analogclock.cpp

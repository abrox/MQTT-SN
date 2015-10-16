TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/LinuxClientSample.cpp \
    src/lib/mqttsn.cpp \
    src/lib/mqttsnClient.cpp \
    src/lib/mqttsnClientAppFw4Linux.cpp \
    src/lib/mqUtil.cpp \
    src/lib/udpStack.cpp \
    src/lib/XXXXXStack.cpp \
    src/lib/zbeeStack.cpp \
    src/lib/mqttsnClientAppFw4Arduino.cpp

HEADERS += \
    src/lib/mqttsn.h \
    src/lib/MQTTSN_Application.h \
    src/lib/mqttsnClient.h \
    src/lib/mqttsnClientAppFw4Linux.h \
    src/lib/mqUtil.h \
    src/lib/Network.h \
    src/lib/udpStack.h \
    src/lib/XXXXXStack.h \
    src/lib/zbeeStack.h \
    src/lib/mqttsnClientAppFw4Arduino.h

OTHER_FILES += \
    README.md


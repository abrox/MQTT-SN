TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/BrokerRecvTask.cpp \
    src/BrokerSendTask.cpp \
    src/ClientRecvTask.cpp \
    src/ClientSendTask.cpp \
    src/GatewayControlTask.cpp \
    src/GatewayResourcesProvider.cpp \
    src/TomyGateway.cpp \
    src/lib/Messages.cpp \
    src/lib/ProcessFramework.cpp \
    src/lib/TCPStack.cpp \
    src/lib/TLSStack.cpp \
    src/lib/Topics.cpp \
    src/lib/UDPStack.cpp \
    src/lib/XXXXXStack.cpp \
    src/lib/ZBStack.cpp

OTHER_FILES += \
    README.md

HEADERS += \
    src/BrokerRecvTask.h \
    src/BrokerSendTask.h \
    src/ClientRecvTask.h \
    src/ClientSendTask.h \
    src/ErrorMessage.h \
    src/GatewayControlTask.h \
    src/GatewayDefines.h \
    src/GatewayResourcesProvider.h \
    src/lib/Defines.h \
    src/lib/Messages.h \
    src/lib/ProcessFramework.h \
    src/lib/TCPStack.h \
    src/lib/TLSStack.h \
    src/lib/Topics.h \
    src/lib/UDPStack.h \
    src/lib/XXXXXStack.h \
    src/lib/ZBStack.h

LIBS += -pthread
LIBS += -L /usr/lib/x86_64-linux-gnu -lrt -lssl -lcrypto


QT -= gui
QT += serialport
QT += serialbus
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        MVCamera/MVCamera.cpp \
        controller.cpp \
        imgprocessthread.cpp \
        inversekinematic.cpp \
        main.cpp \
        minsquresolver.cpp \
        modbuscontroller.cpp \
        motiondetector.cpp \
        pidcontroller.cpp \
        poseestimation2d2d.cpp \
        poseestimation3d2d.cpp \
        serialport.cpp \
        zerodetector.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    GAS_N.h \
    controller.h \
    imgprocessthread.h \
    inversekinematic.h \
    minsquresolver.h \
    modbuscontroller.h \
    motiondetector.h \
    pidcontroller.h \
    poseestimation2d2d.h \
    poseestimation3d2d.h \
    serialport.h \
    timemeasure.h \
    zerodetector.h

INCLUDEPATH += ThirdParty/eigen-eigen-323c052e1731/
INCLUDEPATH += MVCamera\
               MVCamera\DriverInclude
INCLUDEPATH += C:\MySoftware\opencv3.4.3\opencv\build\include\
               C:\MySoftware\opencv3.4.3\opencv\build\include\opencv\
               C:\MySoftware\opencv3.4.3\opencv\build\include\opencv2\
INCLUDEPATH += C:\MySoftware\Matlab\extern\include\

LIBS += ../stewartControl/GAS.lib
LIBS += ../stewartControl/MVCamera/MVCAMSDK_X64.lib
LIBS += C:/MySoftware/opencv3.4.3/opencv/build/x64/vc15/lib/opencv_world343.lib
LIBS += C:/MySoftware/Matlab/extern/lib/win64/microsoft/libMatlabEngine.lib
LIBS += C:/MySoftware/Matlab/extern/lib/win64/microsoft/libMatlabDataArray.lib

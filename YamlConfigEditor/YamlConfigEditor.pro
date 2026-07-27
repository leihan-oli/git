QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# 添加 yaml-cpp 头文件路径（通常系统路径默认已包含，但显式写出更安全）
#INCLUDEPATH += /usr/include

# 链接 yaml-cpp 库
LIBS += -lyaml-cpp

SOURCES += \
    AllConfig.cpp \
    MainWindow.cpp \
    dialogs/BasicSettingsDialog.cpp \
    dialogs/CameraAdjustDialog.cpp \
    dialogs/CameraInfoDialog.cpp \
    dialogs/CameraInfoEditDialog.cpp \
    dialogs/CameraParamsDialog.cpp \
    dialogs/ExposureDialog.cpp \
    dialogs/FeatureDialog.cpp \
    dialogs/FullScreenDialog.cpp \
    dialogs/GazeDialog.cpp \
    dialogs/ScaleDialog.cpp \
    dialogs/SeamDialog.cpp \
    main.cpp

HEADERS += \
    AllConfig.h \
    MainWindow.h \
    dialogs/BasicSettingsDialog.h \
    dialogs/CameraAdjustDialog.h \
    dialogs/CameraInfoDialog.h \
    dialogs/CameraInfoEditDialog.h \
    dialogs/CameraParamsDialog.h \
    dialogs/ExposureDialog.h \
    dialogs/FeatureDialog.h \
    dialogs/FullScreenDialog.h \
    dialogs/GazeDialog.h \
    dialogs/ScaleDialog.h \
    dialogs/SeamDialog.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

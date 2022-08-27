QT += core gui

CONFIG += console c++17 -lfmt

DEFINES += QT_DEPRECATED_WARNINGS
QMAKE_CXXFLAGS_RELEASE += -O3 # Release -O3

message("infantry V 2022.8")

SOURCES += \
    armor_detector.cpp \
    buff_detector.cpp \
    common.cpp \
    coord_solver.cpp \
    data_plot.cpp \
    filter.cpp \
    main.cpp \
    uart_serial.cpp \
    utils.cpp \
    thread_task.cpp \
    video_capture.cpp

HEADERS += \
    armor_detector.h \
    buff_detector.h \
    common.h \
    coord_solver.h \
    data_plot.h \
    filter.h \
    log.h \
    MindVision/CameraApi.h \
    MindVision/CameraDefine.h \
    MindVision/CameraStatus.h \
    obj_manger.h \
    thread_task.h \
    uart_serial.h \
    utils.h \
    Galaxy/DxImageProc.h \
    Galaxy/GxIAPI.h \
    stdafx.h \
    video_capture.h

DISTFILES += \
    configs/car_param.xml \
    configs/filter_param.yml

CONFIG += precompile_header
PRECOMPILED_HEADER = stdafx.h

#---------------OPENCV相关配置---------------

INCLUDEPATH += /usr/local/include/opencv4

LIBS += /usr/local/lib/libopencv_*.so

LIBS += -L/usr/local/lib \
        -lopencv_stitching \
        -lopencv_calib3d \
        -lopencv_features2d \
        -lopencv_objdetect \
        -lopencv_highgui \
        -lopencv_videoio \
        -lopencv_photo \
        -lopencv_imgcodecs \
        -lopencv_video \
        -lopencv_ml \
        -lopencv_imgproc \
        -lopencv_flann \
        -lopencv_core \
        -lopencv_dnn

#---------------相机驱动相关配置---------------

LIBS += /usr/lib/libgxiapi.so \
        /usr/lib/libdximageproc.so \
        -lMVSDK

#-------------------其他配置-------------------
INCLUDEPATH += /usr/include/eigen3 \
               /usr/local/include/ceres
             #  /usr/local/include/fmt

LIBS += -L. -lceres \
        -L. -lblas -llapack -lglog -lgflags -lcholmod -lcxsparse -lX11 -fopenmp \
        -pthread

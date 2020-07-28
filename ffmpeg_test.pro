TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp


#INCLUDEPATH += /usr/local/ffmpeg/include

#LIBS += -lpthread

#LIBS += -L/usr/local/ffmpeg/lib -lavcodec -lavfilter -lavutil -lswresample -lavdevice -lavformat -lpostproc -lswscale


#链接alsa库
LIBS += -lasound -lpthread


#加入ffmpeg头文件并链接ffmpeg库
INCLUDEPATH += /usr/local/ffmpeg/include
LIBS += -L/usr/local/ffmpeg/lib/ -lswscale -lswresample -lavutil -lavformat -lavfilter -lavdevice -lavcodec

#加入opencv头文件并链接opencv库
INCLUDEPATH += /usr/include
LIBS += -L/usr/lib/x86_64-linux-gnu -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_videostab

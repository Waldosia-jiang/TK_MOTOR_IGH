TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

# 编译器设置
QMAKE_CC = gcc
QMAKE_CXX = gcc

# 源文件
SOURCES += \
    igh_example.c \
    motion.c \
    dc.c \
    pdo_set.c \
    trajectory_planning.c

HEADERS += \
    igh_example.h \
    motion.h \
    dc.h \
    pdo_set.h \
    trajectory_planning.h

# 包含路径
INCLUDEPATH += /opt/ethercat/include

# 库路径和库
LIBS += -L/opt/ethercat/lib
LIBS += -lethercat
LIBS += -lpthread

# 编译选项
DEFINES += USE_GNU_THREAD=1

# C标准
QMAKE_CFLAGS += -std=c11

# 输出文件名
TARGET = example

# 排除的文件
SOURCES -= test_mode_switch.c


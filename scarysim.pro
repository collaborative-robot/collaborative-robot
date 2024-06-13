#-------------------------------------------------
#
# Project created by QtCreator 2011-10-02T21:14:46
#
#-------------------------------------------------

QT       += core gui\
            opengl\
            widgets\
            charts

LIBS     += -lglut -lGLU

TARGET = scarysim
TEMPLATE = app


SOURCES += main.cpp\
    functionAlgorithm.c \
    functionFile.c \
        mainwindow.cpp \
    glwidget.cpp \
    model.cpp \
    scara.cpp \
    wgt_manageFile.cpp

HEADERS  += mainwindow.h \
    functionAlgorithm.h \
    functionFile.h \
    glwidget.h \
    model.h \
    scara.h \
    wgt_manageFile.h

FORMS    += mainwindow.ui







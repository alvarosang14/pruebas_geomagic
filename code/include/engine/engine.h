#ifndef ENGINE_H
#define ENGINE_H

#include <QApplication>
#include <thread>
#include <iostream>

#include "position/position.hpp"
#include "qt/mainwindow.h"

class Engine {
private:
    bool runApi(int argc, char *argv[]);
    bool runQt(int argc, char *argv[]);
    bool runRos2(int argc, char *argv[]);

    ApiYarp api;

public:
    Engine() {}

    bool run(int argc, char *argv[]);
};

#endif // ENGINE_H
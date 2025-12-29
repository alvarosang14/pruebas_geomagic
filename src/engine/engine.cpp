#include "engine/engine.h"

bool Engine::run(int argc, char *argv[]) {
    std::thread apiThread(&Engine::runApi, this, argc, argv);
    std::thread qtThread(&Engine::runQt, this, argc, argv);

    qtThread.join();
    api.stopModule();

    return true;
}

bool Engine::runApi(int argc, char *argv[]) {
    // Inicializar YARP
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "Error: YARP network not available" << std::endl;
        return 1;
    }

    // Configurar ResourceFinder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    // Lanzar YARP en un hilo separado
    return api.runModule(rf);
}

bool Engine::runQt(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    MainWindow window;
    window.show();
    
    return app.exec();
}

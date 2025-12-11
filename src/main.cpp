#include "position.hpp"

int main(int argc, char* argv[]) {
    // Inicializar YARP
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "Error" << std::endl;
        return 1;
    }

    // Configurar ResourceFinder y ejecutar
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    Prueba myModule;
    return myModule.runModule(rf);
}
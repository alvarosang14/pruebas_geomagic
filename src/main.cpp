#include "main.hpp"


bool Prueba::configure(yarp::os::ResourceFinder& rf) {
    std::string device = rf.check("device", yarp::os::Value("geomagic")).asString();

    yarp::os::Property optGeo;
    optGeo.put("device", "hapticdeviceclient");
    optGeo.put("remote", ("/" + device).c_str());
    optGeo.put("local", "/cer_teleop/geomagic");

    if (!drvControlSource.open(optGeo)) {
        yError() << "Error abriendo PolyDriver 'hapticdeviceclient'.";
        return false;
    }

    if (!drvControlSource.view(igeo)) {
        yError() << "dynamic_cast a IHapticDevice falló.";
        drvControlSource.close();
        return false;
    }

    yInfo() << "Geomagic conectado y listo.";
    return true;
}

bool Prueba::str_position()  {
    yarp::sig::Vector pos(3);

    igeo->getPosition(pos);
    std::cout << "Posicion: " << pos.toString() << std::endl;

    return true;
}

bool Prueba::updateModule() {
    // Aquí va la lógica que se ejecuta periódicamente
    // Por ejemplo, leer la posición del haptic device
    
    yarp::sig::Vector position;
    if (igeo->getPosition(position)) {
        yInfo() << "Position:" << position.toString();
    }
    
    return true;  // devuelve false para detener el módulo
}

double Prueba::getPeriod() {
    return 0.1;  // se ejecuta cada 0.1 segundos (10 Hz)
}

// Cierra el PolyDriver
bool Prueba::close() {
    drvControlSource.close();
    return true;
}

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
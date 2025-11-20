#include <iostream>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IHapticDevice.h>

#include <yarp/sig/Vector.h>


class Prueba : public yarp::os::RFModule {
private:
    yarp::dev::PolyDriver drvControlSource;
    yarp::dev::IHapticDevice* igeo{nullptr};

public:
    Prueba() {}

    bool configure(yarp::os::ResourceFinder& rf) {
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

    bool str_position()  {
        yarp::sig::Vector pos(3);

        igeo->getPosition(pos);
        std::cout << "Posicion: " << pos.toString() << std::endl;

        return true;
    }

    // Cierra el PolyDriver
    bool close() {
        drvControlSource.close();
        return true;
    }
};

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
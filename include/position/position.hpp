#include <iostream>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IHapticDevice.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Vector.h>


class Prueba : public yarp::os::RFModule {
private:
    yarp::dev::PolyDriver drvControlSource;
    yarp::dev::IHapticDevice* igeo{nullptr};

public:
    Prueba() {}

    bool configure(yarp::os::ResourceFinder& rf);
    bool str_position();
    bool updateModule() override;  // ← Añade esto
    bool close() override;
    double getPeriod() override;
};
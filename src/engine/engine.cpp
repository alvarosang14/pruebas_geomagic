#include "engine/engine.h"

#include <rclcpp/rclcpp.hpp>

bool Engine::run(int argc, char *argv[]) {
    std::thread apiThread(&Engine::runApi, this, argc, argv);
    std::thread qtThread(&Engine::runQt, this, argc, argv);
    std::thread ros2Thread(&Engine::runRos2, this, argc, argv);

    qtThread.join();
    api.stopModule();
    ros2Thread.join();

    return true;
}

bool Engine::runRos2(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("engine_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
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

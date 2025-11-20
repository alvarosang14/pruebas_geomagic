add_executable(prueba_geomagic main.cpp main.hpp)

target_link_libraries(prueba_geomagic PRIVATE
    YARP::YARP_os,
    YARP::YARP_dev)
add_executable(prueba_geomagic main.cpp main.hpp)

target_link_libraries(prueba_geomagic ${YARP_LIBRARIES})
target_include_directories(prueba_geomagic PRIVATE ${YARP_INCLUDE_DIRS})
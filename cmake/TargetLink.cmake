target_link_libraries(prueba_geomagic ${YARP_LIBRARIES})
target_include_directories(prueba_geomagic PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
)
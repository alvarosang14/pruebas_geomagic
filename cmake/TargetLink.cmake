target_link_libraries(prueba_geomagic 
    ${YARP_LIBRARIES}
    Qt6::Core
    Qt6::Widgets
    Qt6::Gui
)

target_include_directories(prueba_geomagic PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
)
cmake_minimum_required(VERSION 3.19...3.20)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON) # for object
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/QtGui)

project(prueba_geomagic LANGUAGES CXX)

if(NOT CMAKE_CONFIGURATION_TYPES)
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    # Establece Release por defecto
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

add_executable(prueba_geomagic 
    src/main.cpp 
    src/position/position.cpp
    src/qt/mainwindow.cpp
    include/qt/mainwindow.h
    QtGui/mainwindow.ui
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(FindLib)
include(TargetLink)

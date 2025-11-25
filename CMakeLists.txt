cmake_minimum_required(VERSION 3.19...3.20)

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

# Busca las dependencias necesarias
find_package(YCM 0.11 REQUIRED)  # Yet Another CMake Module - herramientas auxiliares de CMake
find_package(YARP 3.12 REQUIRED COMPONENTS os dev sig)  # Framework de robótica YARP

# Añade el subdirectorio con el código fuente
add_subdirectory(src)

# Configurar rutas de instalación para recursos YARP
#yarp_configure_external_installation(roboticslab-yarp-devices WITH_PLUGINS)

# Directorios de instalación estándar (bin/, lib/, include/, etc.)
#include(GNUInstallDirs)

# Controlar ubicación de ejecutables y bibliotecas durante la compilación
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Crear targets condicionales
#include(CMakeDependentOption)

# Habilitar soporte para testing con CTest
#enable_testing()

# Registrar el paquete en el registro de usuario de CMake
#set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

# Instalar archivos de configuración del paquete (para find_package)
#include(InstallBasicPackageFiles)
#install_basic_package_files(ROBOTICSLAB_YARP_DEVICES
#                            VERSION 0.1.0
#                            COMPATIBILITY AnyNewerVersion
#                            NO_SET_AND_CHECK_MACRO
#                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
#                            NAMESPACE ROBOTICSLAB::)

# Crear target de desinstalación
#include(AddUninstallTarget)
cmake_minimum_required(VERSION 3.10.2)
project(MobileRackInterface)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include subm/MiniPID subm/fmt/include)

set(BUILD_EXAMPLES true) # this is read by the CMake file in qualisys_cpp_sdk
add_subdirectory(subm/qualisys_cpp_sdk)
add_subdirectory(subm/fmt)

FIND_PACKAGE(PkgConfig)
# https://github.com/AgileManufacturing/Avans/blob/master/cmake/FindModbus.cmake
pkg_check_modules(PC_MODBUS QUIET libmodbus)
set(MODBUS_DEFINITIONS ${PC_LIBMODBUS_CFLAGS_OTHER})

find_path(MODBUS_INCLUDE_DIRS modbus/modbus.h
          HINTS ${PC_LIBMODBUS_INCLUDEDIR} ${PC_LIBMODBUS_INCLUDE_DIRS}
          PATH_SUFFIXES modbus )

find_library(MODBUS_LIBRARIES NAMES modbus libmodbus
             HINTS ${PC_LIBMODBUS_LIBDIR} ${PC_LIBMODBUS_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MODBUS_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MODBUS "Could not find libmodbus" MODBUS_LIBRARIES MODBUS_INCLUDE_DIRS)

FIND_PACKAGE (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

add_library(ValveController SHARED src/ValveController.cpp src/MPA.cpp subm/MiniPID/MiniPID.cpp subm/fmt/src/format.cc)
target_include_directories(ValveController PRIVATE ${MODBUS_DIRS})
target_link_libraries(ValveController -pthread ${MODBUS_LIBRARIES} ${Boost_LIBRARIES})

add_library(QualisysClient SHARED src/QualisysClient.cpp subm/fmt/src/format.cc)
target_link_libraries(QualisysClient qualisys_cpp_sdk -pthread)

add_subdirectory(examples)
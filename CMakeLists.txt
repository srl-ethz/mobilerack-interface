cmake_minimum_required(VERSION 3.10.2)
project(MobileRackInterface)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(include)

### compile the submodules
set(BUILD_EXAMPLES true) # this is read by the CMake file in qualisys_cpp_sdk
add_subdirectory(subm/qualisys_cpp_sdk)

add_subdirectory(subm/fmt)

add_library(MiniPID subm/MiniPID/MiniPID.cpp)
target_include_directories(MiniPID PUBLIC subm/MiniPID)

add_subdirectory(subm/pybind11)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

FIND_PACKAGE(PkgConfig)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_check_modules(MODBUS REQUIRED libmodbus)
pkg_check_modules(SERIALPORT REQUIRED libserialport)

FIND_PACKAGE (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

add_library(ValveController SHARED src/ValveController.cpp src/MPA.cpp)
target_include_directories(ValveController PUBLIC ${MODBUS_INCLUDE_DIRS})
target_link_libraries(ValveController Threads::Threads ${MODBUS_LDFLAGS} MiniPID fmt)

add_library(QualisysClient SHARED src/QualisysClient.cpp)
target_link_libraries(QualisysClient qualisys_cpp_sdk Threads::Threads fmt ${OpenCV_LIBS})

add_library(SerialInterface SHARED src/SerialInterface.cpp)
target_include_directories(SerialInterface PUBLIC ${SERIALPORT_INCLUDE_DIRS})
target_link_libraries(SerialInterface ${SERIALPORT_LDFLAGS} Threads::Threads fmt)

## set up pybind modules
pybind11_add_module(mobilerack_pybind_module src/python_bindings.cpp src/ndarray_converter.cpp)
target_link_libraries(mobilerack_pybind_module PUBLIC QualisysClient ValveController)

add_subdirectory(examples)
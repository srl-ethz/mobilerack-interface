cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 11)
project(SoftTrunk)
include_directories(include src/matplotlib-cpp src/MiniPID include/modbus)

find_package(PythonLibs 2.7)
FIND_PACKAGE(PkgConfig)
# https://github.com/AgileManufacturing/Avans/blob/master/cmake/FindModbus.cmake
pkg_check_modules(PC_LIBMODBUS REQUIRED libmodbus)


FIND_PACKAGE (Eigen3 REQUIRED NO_MODULE)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR})

add_executable(test_valve src/test_valve.cpp src/mpa.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(test_valve PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(test_valve ${PYTHON_LIBRARIES})
target_include_directories(test_valve PRIVATE ${PC_LIBMODBUS_DIRS})
target_link_libraries(test_valve ${PC_LIBMODBUS_LIBRARIES} )


add_executable(example_forceController src/example_forceController.cpp src/forceController.cpp src/mpa.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(example_forceController PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_forceController ${PYTHON_LIBRARIES})
target_link_libraries(example_forceController -pthread)
# http://www.kaizou.org/2014/11/typical-cmake-project/
target_include_directories(example_forceController PRIVATE ${PC_LIBMODBUS_DIRS})
target_link_libraries(example_forceController ${PC_LIBMODBUS_LIBRARIES} )

add_executable(example_sinusoidal src/example_sinusoidal.cpp src/forceController.cpp src/mpa.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(example_sinusoidal PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_sinusoidal ${PYTHON_LIBRARIES})
target_link_libraries(example_sinusoidal -pthread)
# http://www.kaizou.org/2014/11/typical-cmake-project/
target_include_directories(example_sinusoidal PRIVATE ${PC_LIBMODBUS_DIRS})
target_link_libraries(example_sinusoidal ${PC_LIBMODBUS_LIBRARIES} )

FIND_PACKAGE( Boost COMPONENTS program_options system thread REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
ADD_EXECUTABLE(example_optitrack src/example_optitrack.cpp)
TARGET_LINK_LIBRARIES( example_optitrack ${Boost_LIBRARIES} )

# compiling the OptiTrackClient as STATIC(default) causes errors with "multiple definitions"...
ADD_LIBRARY( OptiTrackClient SHARED "src/OptiTrackClient.cpp" )
TARGET_LINK_LIBRARIES( OptiTrackClient ${Boost_LIBRARIES} )

ADD_LIBRARY( CurvatureCalculator SHARED "src/CurvatureCalculator.cpp")
TARGET_LINK_LIBRARIES( CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} OptiTrackClient)

add_executable(example_CurvatureCalculator src/example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_library(arm SHARED src/arm.cpp src/create_urdf.cpp)
target_link_libraries(arm ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

add_executable(create_urdf src/create_urdf.cpp)
target_link_libraries(create_urdf arm)
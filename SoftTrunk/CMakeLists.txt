cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 11)
project(SoftTrunk)
include_directories(include src/matplotlib-cpp src/MiniPID include/modbus)

find_package(PythonLibs 2.7)
FIND_PACKAGE(PkgConfig)
# https://github.com/AgileManufacturing/Avans/blob/master/cmake/FindModbus.cmake
pkg_check_modules(PC_LIBMODBUS REQUIRED libmodbus)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR})

FIND_PACKAGE (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})


add_executable(test_valve src/test_valve.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(test_valve PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(test_valve ${PYTHON_LIBRARIES})
target_include_directories(test_valve PRIVATE ${PC_LIBMODBUS_DIRS})
target_link_libraries(test_valve ${PC_LIBMODBUS_LIBRARIES} )


# http://www.kaizou.org/2014/11/typical-cmake-project/
add_library(ForceController SHARED src/ForceController.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(ForceController PRIVATE ${PC_LIBMODBUS_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(ForceController -pthread ${PC_LIBMODBUS_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example_ForceController src/example_ForceController.cpp)
target_link_libraries(example_ForceController ForceController ${PYTHON_LIBRARIES})
target_include_directories(example_ForceController PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(example_sinusoidal src/example_sinusoidal.cpp)
target_include_directories(example_sinusoidal PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(example_sinusoidal ${PYTHON_LIBRARIES} ForceController -pthread)
# http://www.kaizou.org/2014/11/typical-cmake-project/

FIND_PACKAGE( Boost COMPONENTS program_options system thread REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
ADD_EXECUTABLE(example_optitrack src/example_optitrack.cpp)
TARGET_LINK_LIBRARIES( example_optitrack ${Boost_LIBRARIES} )

# compiling the OptiTrackClient as STATIC(default) causes errors with "multiple definitions"...
ADD_LIBRARY( OptiTrackClient SHARED "src/OptiTrackClient.cpp" )
TARGET_LINK_LIBRARIES( OptiTrackClient ${Boost_LIBRARIES} )

ADD_LIBRARY( CurvatureCalculator SHARED "src/CurvatureCalculator.cpp")
TARGET_LINK_LIBRARIES( CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} OptiTrackClient)

add_library(ControllerPCC src/ControllerPCC.cpp)
target_link_libraries(ControllerPCC ${EIGEN3_LIBRARIES})

add_library(SoftTrunkManager src/SoftTrunkManager.cpp)
target_link_libraries(SoftTrunkManager ${EIGEN3_LIBRARIES})

add_executable(example_CurvatureCalculator src/example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_library(AugmentedRigidArm SHARED src/AugmentedRigidArm.cpp include/SoftTrunkManager.h include/ControllerPCC.h)
target_link_libraries(AugmentedRigidArm ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} ${PYTHON_LIBRARIES})
target_include_directories(AugmentedRigidArm PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(create_xacro src/create_xacro.cpp)
target_link_libraries(create_xacro AugmentedRigidArm ${PYTHON_LIBRARIES})
target_include_directories(create_xacro PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(example_AugmentedRigidArm src/example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm ${PYTHON_LIBRARIES})
target_include_directories(example_AugmentedRigidArm PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(example_ControllerPCC src/example_ControllerPCC.cpp)
target_link_libraries(example_ControllerPCC AugmentedRigidArm ControllerPCC ${PYTHON_LIBRARIES})
target_include_directories(example_ControllerPCC PRIVATE ${PYTHON_INCLUDE_DIRS})
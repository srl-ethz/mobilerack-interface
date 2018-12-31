cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 11)
project(SoftTrunk)
include_directories(include src/MiniPID include/modbus)

find_package(PythonLibs 2.7)
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



# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR})

FIND_PACKAGE (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

FIND_PACKAGE( Boost REQUIRED COMPONENTS program_options system thread REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

# including ROS
# somewhat based on CMakeLists.txt here:
# https://answers.ros.org/question/237494/fatal-error-rosrosh-no-such-file-or-directory/
find_package(catkin QUIET COMPONENTS roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})
# if found, set the CATKIN_FOUND definition, which is read in SoftTrun_common_defs.h to set USE_ROS to true or false.
if (catkin_FOUND)
add_definitions(-DCATKIN_FOUND)
endif()

add_executable(test_valve src/test_valve.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(test_valve PRIVATE)
target_include_directories(test_valve PRIVATE ${MODBUS_DIRS})
target_link_libraries(test_valve ${MODBUS_LIBRARIES} )


# http://www.kaizou.org/2014/11/typical-cmake-project/
add_library(ForceController SHARED src/ForceController.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(ForceController PRIVATE ${MODBUS_DIRS})
target_link_libraries(ForceController -pthread ${MODBUS_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example_ForceController src/example_ForceController.cpp)
target_link_libraries(example_ForceController ForceController)

add_executable(example_sinusoidal src/example_sinusoidal.cpp)
target_link_libraries(example_sinusoidal ForceController -pthread)
# http://www.kaizou.org/2014/11/typical-cmake-project/

# compiling the OptiTrackClient as STATIC(default) causes errors with "multiple definitions"...
ADD_LIBRARY( OptiTrackClient SHARED "src/OptiTrackClient.cpp" )
TARGET_LINK_LIBRARIES( OptiTrackClient ${Boost_LIBRARIES} )

ADD_LIBRARY( CurvatureCalculator SHARED "src/CurvatureCalculator.cpp")
TARGET_LINK_LIBRARIES( CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} OptiTrackClient)

add_library(ControllerPCC SHARED src/ControllerPCC.cpp src/MiniPID/MiniPID.cpp)
target_link_libraries(ControllerPCC ${EIGEN3_LIBRARIES} AugmentedRigidArm SoftArm)

add_library(Manager SHARED src/Manager.cpp)
target_link_libraries(Manager SoftArm AugmentedRigidArm ControllerPCC ${EIGEN3_LIBRARIES} -pthread)

add_executable(example_CurvatureCalculator src/example_CurvatureCalculator.cpp)
target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_library(AugmentedRigidArm SHARED src/AugmentedRigidArm.cpp include/Manager.h include/ControllerPCC.h include/SoftTrunk_common_defs.h)
target_link_libraries(AugmentedRigidArm ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} ${catkin_LIBRARIES})
#${ROSCPP_LIBRARIES} ${SENSOR_MSGS_LIBRARIES})

add_library(SoftArm SHARED src/SoftArm.cpp)
target_link_libraries(SoftArm ForceController CurvatureCalculator)

add_executable(create_xacro src/create_xacro.cpp)
target_link_libraries(create_xacro AugmentedRigidArm)

add_executable(example_AugmentedRigidArm src/example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm)

add_executable(example_ControllerPCC src/example_ControllerPCC.cpp)
target_link_libraries(example_ControllerPCC AugmentedRigidArm ControllerPCC SoftArm)

add_executable(characterize src/characterize.cpp)
target_link_libraries(characterize Manager)

add_executable(experiment src/experiment.cpp)
target_link_libraries(experiment Manager -pthread)


add_executable(example_SoftArm src/example_SoftArm.cpp)
target_link_libraries(example_SoftArm SoftArm -pthread)


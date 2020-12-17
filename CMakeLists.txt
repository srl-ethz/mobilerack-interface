cmake_minimum_required (VERSION 3.10.2)
project(SoftTrunk)

### find and setup for Drake
# cf: https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed
# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

# include(CTest)

if(APPLE)
  set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/opt/python@3.8/bin)
  set(FIND_PYTHON_INTERP_VERSION 3.8)
else()
  set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
  set(FIND_PYTHON_INTERP_VERSION 3.6)
endif()
find_program(PYTHON_EXECUTABLE NAMES python3
  PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
  NO_DEFAULT_PATH
)
find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
  OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED)

get_filename_component(PYTHONPATH
  "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
  REALPATH
)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

### END OF find and setup for Drake

# setup output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include src/MiniPID subm/fmt/include)
set(BUILD_EXAMPLES true) # this is read by the CMake file in qualisys_cpp_sdk
add_subdirectory(subm/qualisys_cpp_sdk)

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

add_executable(test_valve src/test_valve.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(test_valve PRIVATE)
target_include_directories(test_valve PRIVATE ${MODBUS_DIRS})
target_link_libraries(test_valve ${MODBUS_LIBRARIES} )

add_library(ValveController SHARED src/ValveController.cpp src/MPA.cpp src/MiniPID/MiniPID.cpp)
target_include_directories(ValveController PRIVATE ${MODBUS_DIRS})
target_link_libraries(ValveController -pthread ${MODBUS_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example_ValveController src/example_ValveController.cpp)
target_link_libraries(example_ValveController ValveController)

add_executable(example_ValveController_sinusoidal src/example_ValveController_sinusoidal.cpp)
target_link_libraries(example_ValveController_sinusoidal ValveController)

add_library( QualisysClient SHARED src/QualisysClient.cpp subm/fmt/src/format.cc)
target_link_libraries( QualisysClient qualisys_cpp_sdk )

add_executable(example_QualisysClient src/example_QualisysClient.cpp)
target_link_libraries(example_QualisysClient QualisysClient)

# ADD_LIBRARY( CurvatureCalculator SHARED "src/CurvatureCalculator.cpp")
# TARGET_LINK_LIBRARIES( CurvatureCalculator ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} OptiTrackClient)

# add_library(ControllerPCC SHARED src/ControllerPCC.cpp src/MiniPID/MiniPID.cpp)
# target_link_libraries(ControllerPCC ${EIGEN3_LIBRARIES} AugmentedRigidArm SoftTrunkInterface)

# add_library(Manager SHARED src/Manager.cpp)
# target_link_libraries(Manager SoftTrunkInterface AugmentedRigidArm ControllerPCC ${EIGEN3_LIBRARIES} -pthread)

# add_executable(example_CurvatureCalculator src/example_CurvatureCalculator.cpp)
# target_link_libraries(example_CurvatureCalculator CurvatureCalculator)

add_library(AugmentedRigidArm SHARED src/AugmentedRigidArm.cpp)
target_link_libraries(AugmentedRigidArm drake::drake ${catkin_LIBRARIES})

# add_library(SoftTrunkInterface SHARED src/SoftTrunkInterface.cpp)
# target_link_libraries(SoftTrunkInterface ValveController CurvatureCalculator)

add_executable(create_urdf src/create_urdf.cpp subm/fmt/src/format.cc)

add_executable(example_AugmentedRigidArm src/example_AugmentedRigidArm.cpp)
target_link_libraries(example_AugmentedRigidArm AugmentedRigidArm)

# add_executable(example_ControllerPCC src/example_ControllerPCC.cpp)
# target_link_libraries(example_ControllerPCC AugmentedRigidArm ControllerPCC SoftTrunkInterface)

# add_executable(characterize_part2 src/characterize_part2.cpp)
# target_link_libraries(characterize_part2 Manager)

# add_executable(characterize_part1 src/characterize_part1.cpp)
# target_link_libraries(characterize_part1 Manager)

# add_executable(experiment src/experiment.cpp)
# target_link_libraries(experiment Manager -pthread)


# add_executable(example_SoftTrunkInterface src/example_SoftTrunkInterface.cpp)
# target_link_libraries(example_SoftTrunkInterface SoftTrunkInterface -pthread)


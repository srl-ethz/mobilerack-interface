# Fabrication of Physical Soft Robot
See **Soft Robot Fabrication.md**

# How to set up software for new arm
1. set parameters
    1. Parameters of arm is defined in *SoftTrunk_common_defs.h* in include/ directory.
    1. Modify AugmentedRigidArm.cpp to match the length and mass of each PCC element with the physical arm. (TODO: move this to SoftTrunk_common_defs as well)
1. create URDF files
    1. run ./create_xacro in SoftTrunk/ directory to create the URDF XACRO(URDF but with macros) files for robot, *robot.urdf.xacro*, inside urdf/ directory.
    1. run ./create_urdf.sh in /urdf directory to create the URDF file, robot.urdf (which combines robot.urdf.xacro and macro_definitions.urdf.xacro). 
    This requires the [ROS XACRO](http://wiki.ros.org/xacro) package. (This package should be included in a standard [ROS installation](http://wiki.ros.org/kinetic/Installation/Ubuntu). If not, set up the sources.list and keys as written on the webpage, then `sudo apt install ros-kinetic-xacro`).
    1. The generated URDF file can be previewed with `roslaunch rviz.launch` in the /urdf directory
        1. add RobotModel, and choose base_frame as fixed frame to view robot. (By pressing ctrl-s in RVIZ, this configuration will be saved and you won't have to do it again)
        ![](./img/rviz.png)
1. run experiment

# OS
Works on Ubuntu and macOS. On macOS, the ROS features are unavailable- therefore, the XACRO -> URDF conversion must be done on a Linux machine with ROS installed. It is also impossible to preview the URDF with Rviz.

# Installing necessary libraries

## libmodbus
Communication to FESTO valve array via mod bus 
```
sudo apt install libmodbus-dev
```

On a Mac, install libmodbus [from source](https://libmodbus.org/download/).

## NatNetLinux
This linux library is needed for listening to a udp communication from Optitrack Motive 1.10.0 on a windows machine and streaming rigid bodies. Ensure you install the [Prerequisites](https://github.com/rocketman768/NatNetLinux) for NatNetLinux.
In the /3rd directory, NatNetLinux is added as a submodule. After fetching the submodule with:
```
git submodule init
git submodule update
```
compile and install NatNetLinux:
```
mkdir build
cd build
cmake ../NatNetLinux
make
sudo make install
```
## Eigen3
[Get the code](http://eigen.tuxfamily.org/index.php?title=Main_Page), unzip, navigate into the unzipped folder, prepare with cmake and then install this header libary:
```
mkdir build
cd build/
cmake ..
sudo make install
```

## RBDL
This code uses the [Rigid Body Dynamics Library](https://rbdl.bitbucket.io/index.html) for calculating the dynamics of the rigid bodies of the augmented robot model.
Download the most recent stable version as zip file, then follow its README to install, but make sure to also compile the URDF reader addon, see below:
```
mkdir build
cd build/
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON CMAKE_BUILD_TYPE=Release ../
make
sudo make install
```
You need to install Eigen3 before, as described above.

# Libraries
each library usually has a demo program, just compile the library with `cmake .; make`.
## SoftTrunkManager
The topmost class for the SoftTrunk robot system. Has instances of AugmentedRigidArm, ControllerPCC, and SoftArm and coordinates them to control the robot. curvatureControl() can be called to do dynamic curvature control on the robot, and characterize() can be called to run experiments to characterize the parameters alpha, k, and d of the soft arm.

## SoftArm
Represents the physical, soft arm robot. Implements the CurvatureCalculator library to get the current state of the robot, and the ForceController to actuate it. The actuate() function can be called to actuate the arm, its input being the tau_pt(i.e. the torque in phi-theta coordinates).

## OptiTrackClient
Talks to Motive software over UDP, receives current information for each RigidBody which can then be used in CurvatureCalculator.cpp (see below).

## CurvatureCalculator
Calculates the curvature of each soft arm segment based on the OptiTrack measurements of the base and tip of a segment. This has not been tested on the actual system yet.

example_CurvatureCalculator.cpp is the sample code using this library - This demo prints out for each segment of the soft arm the rotation around the center axis called phi, and the calculated degree of curvature called theta. The vector representing the configuration, q, is structured to be {(phi of root section), (theta of root section), (phi of next section), ...}.

## ForceController
Implements an individual PID control for each valve of the FESTO valve array.

example_sinusoidal.cpp and example_forceController.cpp is a demo of this library. The former sends out sinusoidal signals to each compartment.

## AugmentedRigidArm
Represents the kinematic & dynamic info about the augmented rigid arm model(which approximates the soft arm), and computes its kinematics/dynamics using RBDL. It can also generate XACRO files based on the parameters of the robot.

example_ControllerPCC.cpp creates an instance of AugmentedRigidArm, and inputs the q and dq to get various kinematic / dynamic information of the arm.
## ControllerPCC
Implements the PCC controller, as described in the paper. It receives pointers to instances of AugmentedRigidArm and SoftArm, so it can access instances of those classes to retrieve information about them and use it to control the Soft Trunk. By setting USE_PID_CURVATURE_CONTROL to true in SoftTrunk_common_defs.h, it can also do PID control.

example_SoftArm.cpp creates an AugmentedRigidArm and SoftArm, and computes the torque required for curvature dynamic control.

# programs
## create_xacro
Uses the arm library to generate the *robot.urdf.xacro* file, and saves it to the urdf/ directory.

## characterize
Runs the characterize() function in SoftTrunkManager to characterize the parameters of the robot. Not fully implemented- At this moment, it just sends out step signals to the arm.

## experiment
Actuates the arm using the controller. Computes some curvature profile, then sends each one to SoftArmManager with a set time interval to actuate it. Doesn't really work now; the arm flails around chaotically when this is run. must debug the entire code to brush it up...

# comments in code
As suggested [here](https://softwareengineering.stackexchange.com/questions/84071/is-it-better-to-document-functions-in-the-header-file-or-the-source-file),
* **how to use** the function / class / variable will be commented in the header file
* **how the code works** will be commented in the source code

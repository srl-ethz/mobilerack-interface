# Soft Trunk repository
# Documentation
## [Overleaf](https://www.overleaf.com/read/rsnsrjphmpxx)
There is a report on Overleaf detailing how the C++ code is implemented, and how it has been changed since the RoboSoft paper.

[![](./img/report.png)](https://www.overleaf.com/read/rsnsrjphmpxx)
## Doxygen
Uses Doxygen to generate documentation from inline comments in code.
Install [Doxygen](http://www.doxygen.nl), and run `doxygen` in this directory (`<3d_soft_trunk_contact_repo>/SoftTrunk`) to generate HTML (can be seen with browser at html/index.html) & LATEX output.
 
[GraphViz](https://www.graphviz.org/download/) is required if you want to generate dependency graphs. 

This is the "collaboration diagram" for the Manager class(topmost class for control of the robot):
![](./img/collaboration_diagram.png)

A good starting point to read the code is experiment.cpp, the Manager class, and SoftTrunk_common_defs.h

# Fabrication of Physical Soft Robot
See **Soft Robot Fabrication.md**

# Bill of materials
See **Bill of materials.csv**

# Compiling
First, make sure that 
In this directory (`<3d_soft_trunk_contact_repo>/SoftTrunk`), run `$ cmake .` to generate the Makefile. Then, compile the program you want to run, e.g.`$ make experiment`. Then run it, e.g. `$ ./experiment`. 

# Setting up a new arm
1. Fix arm in place, making sure to align the chambers with the X and Y axes in Motive.
![]()
1. Connect tubes. The mapping from each valve ID to chamber ID can be defined later.
1. Put marker rings at the end of each segment.
![]()
1. Define RigidBodies in Motive.
    1. Set the **User ID**for the base element's frame to **0**, the next frame's ID to **1**, and so on.
    ![]() 
    1. Use the **transformation** tool to translate the center of the base frame to the center of the 4 main markers, and translate the other frames so that the z axis forms one line.   
    ![]()
    1. Make sure Motive is streaming data (should be on by default: check View → Data Streaming)
1. set parameters
    1. Go through *SoftTrunk_common_defs.h* in include/ directory and change parameters as needed. (for determining VALVE_MAP, it may be helpful to use example_ForceController to actuate each chamber)
1. create URDF files
    1. Some URDF files are pre-prepared in 2_segment_urdf and 3_segment_urdf in urdf/, so that you won't have to use ROS. Copy the appropriate urdf file to the /urdf directory.
    1. If there is no appropriate URDF file available, do the next steps.
    1. run ./create_xacro in SoftTrunk/ directory to create the URDF XACRO(URDF but with macros) files for robot, *robot.urdf.xacro*, inside urdf/ directory. This creates a file based on the parameters defined in SoftTrunk_common_defs.h. 
    1. run ./create_urdf.sh in /urdf directory to create the URDF file, robot.urdf (which combines robot.urdf.xacro and macro_definitions.urdf.xacro). 
    This requires the [ROS XACRO](http://wiki.ros.org/xacro) package. (This package should be included in a standard [ROS installation](http://wiki.ros.org/kinetic/Installation/Ubuntu). If not, set up the sources.list and keys as written on the webpage, then `sudo apt install ros-kinetic-xacro`).
    1. The generated URDF file can be previewed with `roslaunch rviz.launch` in the /urdf directory
        1. add RobotModel, and choose base_frame as fixed frame to view robot. (By pressing ctrl-s in RVIZ, this configuration will be saved and you won't have to do it again)
        ![](./img/rviz.png)

# Characterization
This is required before being able to use the dynamic controller.

## Part 1
1. Attach a string to the tip of the arm, so it can be pulled at a constant, known force.
1. run characterize_part1. **The arm must be straight for the first 2 seconds**, so it can calibrate itself (process to remove initial offset value). Do not pull the string until the output prints `Setup of Manager done.`.
1. A matrix equation will be printed for you to solve, and get values for alpha. (the values in the direction you're not pulling is quite meaningless)

## Part 2
1. Input the alpha values derived from part 1 into SoftTrunkInterface.cpp.
1. run characterize_part2.

# Viewing logs


# OS Notes
Works on Ubuntu and macOS. On macOS, the ROS features are unavailable- therefore, the XACRO -> URDF conversion must be done on a Linux machine with ROS installed. It is also impossible to preview the URDF with Rviz. (some pre-prepared urdf files are available in the /urdf directory.)
On macOS, install [homebrew](https://brew.sh/) or if already installed, run `brew update`.

# Installing necessary libraries
## boost
boost extends C++ features. On Ubuntu, install from [boost](https://www.boost.org/) or use 
```
sudo apt-get install libboost-all-dev
```

On macOS, do `brew install boost`

## pkg-config
helps manage libraries or something. On macOS,
```
brew install pkg-config
```
Note: On ubuntu, it is possibly already installed.

## libmodbus
Used for communication to FESTO valve array via modbus. For Ubuntu, 
```
echo "for Ubuntu"
sudo apt install libmodbus-dev

echo "for macOS"
brew install libmodbus
```

## NatNetLinux
This linux library is needed for listening to a udp communication from Optitrack Motive 1.10.0 on a windows machine and streaming rigid bodies. Ensure you install the [Prerequisites](https://github.com/rocketman768/NatNetLinux) for NatNetLinux.
For MacOS, ensure you are on os version 10.12 or newer.
In the /3rd directory(which is in the root of this repository), NatNetLinux is added as a submodule. After fetching the submodule with:
```
cd <3d_soft_trunk_contact_repo>
git submodule init
git submodule update
```
compile and install NatNetLinux. In the /build directory of the repository, run:
```
cd <3d_soft_trunk_contact_repo>/3rd
mkdir build
cd build
cmake ../NatNetLinux
make
sudo make install
```
## Eigen3
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is the math library used.
[Get the code version 3.3.7](http://bitbucket.org/eigen/eigen/get/3.3.7.zip), unzip, navigate into the unzipped folder, prepare with cmake and then install this header libary:
```
mkdir build
cd build/
cmake ..
sudo make install
```
[This](https://eigen.tuxfamily.org/dox/GettingStarted.html) is a good starting reference for Eigen.

## RBDL
This code uses the [Rigid Body Dynamics Library](https://rbdl.bitbucket.io/index.html) for calculating the dynamics of the rigid bodies of the augmented robot model.
Download the most recent stable version as zip file, we tested with 2.6.0. Then follow its README to install, but make sure to also compile the URDF reader addon. Run in rbdl's directory:
```
mkdir build
cd build/
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON CMAKE_BUILD_TYPE=Release ../
make
sudo make install
```
You need to install Eigen3 before installing RBDL.

# Install this repository (3D Soft Trunk Contact)

```
cd <3d_soft_trunk_contact_repo>/SoftTrunk
cmake .
make
```

# Experiment Setup

## Ethernet Connection.
Connect Controller PC with software installed to the router (dlink-EADA) that is physically connected via ethernet to the Festo Valve Terminal and also to the windows machine running Motive (Optitrack cameras).

## Motive
Power on windows Machine and start the Motive Project that has the calibrated cameras and contains the number of segments plus 1 as rigid bodies to be streamed. IP of the Motive machine should most likely be 192.168.1.105.

## Festo valves
Power up the valves and make sure the terminal is connected to the router.


# Fabrication of soft robot
View **Soft Robot Fabrication.md**

# libraries
each library usually has a demo program, just compile it with `cmake .; make`.
## OptiTrackClient.cpp
Talks to Motive software over IP, receives current information for each RigidBody which can then be used in CurvatureCalculator.cpp.

## CurvatureCalculator.cpp
calculates the curvature of each piece based on OptiTrack measurements. (not tested on actual thing yet)

example_CurvatureCalculator.cpp is the sample code using this library- Prints out calculated theta and phi for each segment.

## forceController.cpp
Implements indivisual PID control for each valve.

example_sinusoidal.cpp and example_forceController.cpp is a demo of this library. The former sends out sinusoidal signals to each compartment.

## arm.cpp
Supposed to consolidate all the kinematic & dynamic info about the arm, but still a work in progress.

# programs
(only libraries and demo for each library has been created, no programs that combine the libraries yet.)


# Installing necessary libraries
## libmodbus
`sudo apt install libmodbus-dev`
## RBDL
This code uses the [Rigid Body Dynamics Library](https://rbdl.bitbucket.io/index.html).
Download the most recent stable version as zip file, then follow its README to install, but make sure to also compile the URDF reader addon;
```
mkdir build
cd build/
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON CMAKE_BUILD_TYPE=Release ../
make
sudo make install
```
May also need to install Eigen3, as described.

## NatNetLinux
In the /3rd directory, NatNetLinux is added as a submodule. After fetching the submodule with `git submodule update`, compile and install:
```
mkdir build
cd build
cmake ../NatNetLinux
make
sudo make install
```
## Eigen
[Get the code](http://eigen.tuxfamily.org/index.php?title=Main_Page), and install(make build directory, `cmake ..`, `make`, `sudo make install`).

# comments in code
As suggested [here](https://softwareengineering.stackexchange.com/questions/84071/is-it-better-to-document-functions-in-the-header-file-or-the-source-file),
* **how to use** the function / class / variable will be commented in the header file
* **how the code works** will be commented in the source code

# Fabrication of soft robot
View **Soft Robot Fabrication.md**
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

# comments in code
As suggested [here](https://softwareengineering.stackexchange.com/questions/84071/is-it-better-to-document-functions-in-the-header-file-or-the-source-file),
* **how to use** the function / class / variable will be commented in the header file
* **how the code works** will be commented in the source code

# Installing RBDL
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

# Mobile Rack Interface

Code to connect to the experimental equipment on the Mobile Rack workbench, such as Festo valves & Qualisys motion
tracking system. Code common across different projects using THE RACK can be kept here.

## set up WSL (for Windows)
1. Get [Ubuntu 18.04 from the Microsoft Store](https://www.microsoft.com/store/productId/9N9TNGVNDL3Q). If you don't need GUI, no further steps needed.

### suggested tutorials
* https://docs.microsoft.com/en-us/learn/modules/get-started-with-windows-subsystem-for-linux/
* https://ubuntu.com/tutorials/command-line-for-beginners

### to enable GUI in WSL
1. Make sure you're running WSL **2** (check by running `wsl -l -v` in Windows command line), if on WSL **1**, [refer to this](https://docs.microsoft.com/en-us/windows/wsl/install-win10) and update to WSL 2.
1. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/). This will be used for X11 forwarding in order to use GUI.
1. Launch VcXsrv with settings: *Multiple windows* -> *Start no client* -> check all except *Native opengl*
1. add to end of ~/.bashrc
    ```bash
    export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf 2>/dev/null):0
    export LIBGL_ALWAYS_INDIRECT=0
    ```
    and open new terminal or run `source ~/.bashrc`.
1. GUI should work now! Try it out with `xeyes`, `xcalc`, `xclock` etc (need to install with `sudo apt install x11-apps`) (reboot may be needed).
1. For some PCs, by checking *Native opengl* in VcXsrv and adding `export LIBGL_ALWAYS_INDIRECT=1` to ~/.bashrc, OpenGL can be used.

## get this repository

In the desired directory, run...
```bash
## option 1: clone with https- enter username & password each time you access remote
git clone --recursive https://gitlab.ethz.ch/srl/mobilerack-interface.git 
```
or
```bash
## option 2: clone with SSH- need to set up SSH key in GitLab, no username / password required
git clone --recursive git@gitlab.ethz.ch:srl/mobilerack-interface.git
```
(`--recursive` option will automatically clone the submodules as well)

## install necessary packages
(also check .gitlab-ci.yml for hints on how to setup Ubuntu)

for Ubuntu

```bash
sudo apt update
sudo apt install cmake libmodbus-dev libeigen3-dev libserialport-dev
sudo apt install python3-dev  # install developer package for system's default python version.
```

for macOS (todo: unverified)

```bash
brew install libmodbus eigen libserialport
```

## Compile

```bash
cd /path/to/mobilerack-interface
cmake .
make
```

executables are output to bin/, libraries are output to lib/.

## Python interface
In its current implementation, you must set the `$PYTHONPATH` environment variable to point to the directory containing the library binaries in order to run. (probably `mobilerack-interface/lib`)

```bash
## run this everytime you open a new terminal to run a python script using this library
export PYTHONPATH=$PYTHONPATH:/path/to/lib
## Alternatively, append the line to ~/.bashrc if you don't want to run it every time.
python3
>> import mobilerack_pybind_module
>> vc = mobilerack_pybind_module.ValveController("192.168.0.100", [0, 1], 200)
>> vc.setSinglePressure(0, 100)
```

## Generating Documentation

Uses Doxygen to generate documentation from inline comments in code. Install [Doxygen](http://www.doxygen.nl), and
run `doxygen Doxyfile` in this directory to generate HTML files (can be seen with browser at html/index.html).

## setup for Qualisys Track Manager (QTM)

The QualisysClient class tracks each RigidBody (labels must be an integer between 0 - 9 in order for the QualisysClient to read it).
Before running sample program, update the IP address of the PC running QTM, in examples/example_QualisysClient.cpp.
When on WSL, set IP address to be the IPv4 address of *vEthernet (WSL)*, seen in **Settings** -> **Network&Internet** -> **View your network properties**.

## Festo valves

Power up the valves and make sure your PC is connected to the router.

## Incorporate this into your project

refer to [3d-soft-trunk](https://gitlab.ethz.ch/srl/3d-soft-trunk) as an example.

## references
* https://medium.com/javarevisited/using-wsl-2-with-x-server-linux-on-windows-a372263533c3
* https://stackoverflow.com/questions/61110603/how-to-set-up-working-x11-forwarding-on-wsl2

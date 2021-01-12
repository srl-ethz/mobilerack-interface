# Mobile Rack Interface

Code to connect to the experimental equipment on the Mobile Rack workbench, such as Festo valves & Qualisys motion
tracking system. Code common across different projects using THE RACK can be kept here.

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

executables are output to bin/.

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
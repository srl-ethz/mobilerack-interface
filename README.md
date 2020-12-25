# Mobile Rack Interface

Code to connect to the experimental equipment on the Mobile Rack workbench, such as Festo valves & Qualisys motion
tracking system. Code common across different projects using THE RACK can be kept here.

## Generating Documentation

Uses Doxygen to generate documentation from inline comments in code. Install [Doxygen](http://www.doxygen.nl), and
run `doxygen` in this directory to generate HTML (can be seen with browser at html/index.html) & LATEX output.
[GraphViz](https://www.graphviz.org/download/) is required if you want to generate dependency graphs.

## get submodules

if the submodules (in subm/ directory) are empty, get them with

```bash
git submodule init
git submodule update
```

## install necessary packages

for Ubuntu

```bash
sudo apt install libmodbus-dev libeigen3-dev
```

for macOS (todo: unverified)

```bash
brew install libmodbus
# also install eigen
```

## Compile

```bash
cd /path/to/mobilerack-interface
cmake .
make
```

executables are output to bin/.

## setup for Qualisys Track Manager

todo...

## Festo valves

Power up the valves and make sure it is connected to the router.
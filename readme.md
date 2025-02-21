# Simox

## Disclaimer

This fork of Simox is maintained by the Institute for Artificial Intelligence (IAI) at the University of Bremen.
The original repository is hosted at the H2T GitLab at the Karlsruhe Institute of Technology (KIT) and can be found [here](https://git.h2t.iar.kit.edu/sw/simox/simox).

## About Simox

The aim of the lightweight platform independent C++ toolbox Simox is to provide a set of
algorithms for 3D simulation of robot systems, sampling based motion planning and grasp planning.
Simox consists of three libraries (Virtual Robot, Saba and Grasp Studio) and numerous
examples showing how these libraries can be used to build complex tools in the
context of mobile manipulation.

**Simox Utility** is a utility library providing general-purpose code tools to help working with,
among others,
C++ strings and containers,
mathematics (e.g. pose, radians and degrees, scaling, clamping, periodic mean, ...),
data file formats (e.g. JSON, XMl), colors and color maps,
as well as shapes like axis-aligned and oriented (bounding) boxes.

The library **Virtual Robot** can be used to define complex robot systems,
which may cover multiple robots with many degrees of freedom.
The robot structure and its visualization can be easily defined via XML files and environments with
obstacles and objects to manipulate are supported.
Further, basic robot simulation components, as Jacobian computations and generic
Inverse Kinematics (IK) solvers, are offered by the library.
Beyond that, extended features like tools for analyzing the reachable workspace for robotic manipulators
or contact determination for grasping are included.

With **Saba**, a library for planning collision-free motions is offered, which directly incorporates
with the data provided by Virtual Robot.
The algorithms cover state-of-the-art implementations of sampling-based motion planning approaches
(e.g. Rapidly-exploring Random Trees) and interfaces that allow to conveniently implement own planners.
Since Saba was designed for planning in high-dimensional configuration spaces, complex planning problems
for robots with a high number of degrees of freedom (DoF) can be solved efficiently.

**Grasp Studio** offers possibilities to compute the grasp quality for generic end-effector
definitions, e.g. a humanoid hand.
The implemented 6D wrench-space computations can be used to easily (and quickly) determine
the quality of an applied grasp to an object.
Furthermore, the implemented planners are able to generate grasp maps for given objects automatically.

Since complex frameworks have to incorporate with several libraries in order to provide full functionality,
several issues may arise when setting up the environment, such as dependency problems,
incompatible library versions or even non-existing ports of needed libraries for the used operating systems.
Hence, only a limited set of libraries are used by the Simox core in order to make it compile.
Extended functionality (e.g. visualization) can be turned off in order to allow Simox compiling on most platforms.
Further dependencies are encapsulated with interfaces, making it easy to exchange
e.g. the collision engine or the visualization functionality.
As a reference implementation, Simox offers Coin3D/SoQt-based visualization support.

## Installation

This installation is tested on Ubuntu 20.04. The simox library needs to be installed form source.

### Xstow

If you want to use xstow to install the dependencies from source follow these steps first. Xstow allows you to manage package installations easily and keep the installations of multiple packages separate, while making them appear to be installed in the same location. You can also install packages in the home directory and remove them cleanly if necessary.

If you decide not to use xstow, `make install' will require root privileges and will install the packages in `/usr/local`.

```bash
sudo apt instal xstow
```

Add the following to you `${HOME} /.bashrc`.

```bash
export PATH=${HOME}/local/bin:${HOME}/local/sbin:${HOME}/local/usr/bin:${PATH}
export LD_LIBRARY_PATH=${HOME}/local/lib:${HOME}/local/usr/lib:${LD_LIBRARY_PATH}
export LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}
export CPATH=${HOME}/local/include:${HOME}/local/usr/include:${CPATH}
export LDFLAGS="-L${HOME}/local/lib ${LDFLAGS}"
export PKG_CONFIG_PATH=${HOME}/local/lib/pkgconfig
export CMAKE_INCLUDE_PATH=${CPATH}
export CMAKE_LIBRARY_PATH=${LIBRARY_PATH}

export PYTHON_VERSION=$(basename $(readlink -e $(which python2)))
export PYTHON3_VERSION=$(basename $(readlink -e $(which python3)))
export PYTHONPATH=${HOME}/local/lib/python/site-packages:${HOME}/local/lib/python3/site-packages:${HOME}/local/lib/${PYTHON_VERSION}/site-packages:${HOME}/local/lib/${PYTHON_VERSION}/dist-packages:${HOME}/local/lib/${PYTHON3_VERSION}/site-packages:${HOME}/local/lib/${PYTHON3_VERSION}/dist-packages:${HOME}/local/lib/python/dist-packages:${HOME}/local/lib/python3/dist-packages:${PYTHONPATH}
```

Finally create the directories

```bash
mkdir -p local/src
mkdir -p local/DIR
```

### Install dependencies

First make sure you have these dependencies installed in your system:

- CMake >=2.8.3: A cross-platform, open-source build system. http://www.cmake.org/
- Boost >=1.42: Provides free peer-reviewed portable C++ source libraries. http://www.boost.org
- Eigen >=3.0: A header-only C++ template library for linear algebra: matrices, vectors, and numerical solvers. http://eigen.tuxfamily.org

Then install:

- nlohmann-json, needs to be installed from source on Ubuntu 20.04
- libcoin-dev
- libsoqt520-dev
- libpugixml-dev
- doxygen
- libnlopt-dev
- libnlopt-cxx-dev
- rbdl, needs to be installed from source
- pugixml, due to issues during the compilation, this needs to be installed from source

```bash
sudo apt install libcoin-dev libsoqt520-dev doxygen libnlopt-dev libnlopt-cxx-dev
```

Install `nlohmann-json` , `rbdl` and `pugixml` from source:

```bash
cd ${HOME}/local/src
git clone https://github.com/nlohmann/json.git nlohmann-json/
cd nlohmann-json/
mkdir build/
cd build/
ccmake ..  # configure CMAKE_INSTALL_PREFIX to /home/[user]/local/DIR/nlohmann-json
make
make install  # or sudo make install, if you are not using xstow
# in case of xstow, remember to configure the package
cd ../../../DIR
xstow nlohman-json
```

```bash
git clone https://github.com/rbdl/rbdl.git
cd rbdl/
mkdir build/
cd build/
ccmake ..  # configure CMAKE_INSTALL_PREFIX to /home/[user]/local/DIR/rbdl
make
make install  # or sudo make install, if you are not using xstow
# in case of xstow, remember to configure the package
cd ../../../DIR
xstow rbdl (install the binaries etc.)
```

```bash
git clone https://github.com/zeux/pugixml.git
cd pugixml/
mkdir build/
cd build/
ccmake ..  # configure CMAKE_INSTALL_PREFIX to /home/[user]/local/DIR/pugixml
make
make install  # or sudo make install, if you are not using xstow
# in case of xstow, remember to configure the package (install the binaries etc.)
cd ../../../DIR
xstow pugixml
```

And now we can build simox itself.

```bash
git clone https://git.h2t.iar.kit.edu/sw/simox/simox.git
cd simox/
mkdir build/
cd build/
ccmake ..  # configure CMAKE_INSTALL_PREFIX to /home/[user]/local/DIR/simox
make
make install  # or sudo make install, if you are not using xstow
# in case of xstow, remember to configure the package (install the binaries etc.)
cd ../../../DIR
xstow simox
```

## Documentation

Wiki: https://git.h2t.iar.kit.edu/sw/simox/simox/-/wikis/home

API Reference: http://simox.sourceforge.net/documentation

## License

GNU Lesser General Public License, version 2.1 or any later version.
(see license.txt)

## Copyright
2010-2016 Nikolaus Vahrenkamp
 
## Reference
N. Vahrenkamp, M. Kröhnert, S. Ulbrich, T. Asfour, G. Metta, R. Dillmann  and G. Sandini,
Simox: A Robotics Toolbox for Simulation, Motion and Grasp Planning,
International Conference on Intelligent Autonomous Systems (IAS),
pp. 585 - 594, 2012

```
@INPROCEEDINGS {Vahrenkamp12,
    author = {Nikolaus Vahrenkamp and Manfred Kr\"ohnert and Stefan Ulbrich and Tamim Asfour and Giorgio Metta and R\"udiger Dillmann and Giulio Sandini},
    title = {Simox: A Robotics Toolbox for Simulation, Motion and Grasp Planning},
    booktitle = {International Conference on Intelligent Autonomous Systems (IAS)},
    pages = {585--594},
    year = {2012}
}
```

## Contact
The maintainer of this fork is the IAI at the University of Bremen.\
Contact: Jeroen Schäfer, [jeroen.schaefer@uni-bremen.de](mailto:jeroen.schaefer@uni-bremen.de).

Original Authors:

Nikolaus Vahrenkamp
vahrenkamp at kit dot edu

Repository:
https://git.h2t.iar.kit.edu/sw/simox/simox

Mailing list:
https://groups.google.com/d/forum/simox

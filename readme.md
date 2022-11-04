# Simox

The aim of the lightweight platform independent C++ toolbox Simox is to provide a set of
algorithms for 3D simulation of robot systems, sampling based motion planning and grasp planning.
Simox consists of three libraries (Virtual Robot, Saba and Grasp Studio) and numerous
examples showing how these libraries can be used to build complex tools in the
context of mobile manipulation.

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
Nikolaus Vahrenkamp
vahrenkamp at kit dot edu

Repository:
https://git.h2t.iar.kit.edu/sw/simox/simox

Mailing list:
https://groups.google.com/d/forum/simox

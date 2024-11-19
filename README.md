# POLIMIcontrol-lib #

control-lib is a C++ library that implements different tools to support the development and use of controllers and control structures.

The library includes:

* continuous_ss, a C++ class that implements the state-space form of a linear time-invariant/time-variant/LPV continuous time system
* discrete_ss, a C++ class that implements the state-space form of a linear time-invariant/time-variant/LPV discrete time system
* discrete_tf, a C++ class that implements the transfer function of a discrete time system
* discrete_FIR, a C++ class that implements the transfer function of a discrete time FIR filter
* discrete_integrator, a C++ class that implements the transfer function of a discrete integrator using Forward Euler, Backward Euler or Trapezoidal method
* discrete_derivative, a C++ class that implements the transfer function of a discrete derivative
* differentiator, a C++ class that implements a robust exact differentiator

and the following controllers:

* PID
* ISA PID
* Advanced Gain Scheduling

### How to compile this library ###

The library can be easily compiled using Cmake 
```
cd POLIMIcontrol-lib
mkdir build
cd build
cmake ..
make
```
and then installed in your system folders with
```
sudo make install
```

### How to use this library ###

To use the library in a Cmake environment add the following lines to the CMakeLists.txt
```
find_package(POLIMIcontrol REQUIRED)
target_link_libraries(<target> POLIMIcontrol)
```

For example, to call the library from foo.cpp and generate foo target:
```
find_package(POLIMIcontrol REQUIRED)
add_executable(foo src/foo.cpp)
target_link_libraries(foo POLIMIcontrol)
```

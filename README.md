# gnss_gtsam

The purpose of this library is to create measurements for GNSS (global navigation satellite systems) that can be used to solve estimation problems using GTSAM.  While a couple of basic measurements will be provided, it is also meant to be small and simple to show how to create a custom GTSAM library so other measurements (factors) can easily be added.  Also, will make sure everything can be bound to Python using the [borglab/wrap](https://github.com/borglab/wrap) library (same library used by GTSAM to do Python bindings).  This library builds on pybind.

## Measurements (hopefully updated)
The following "factors" are implemented in this library
* pseudo-range
* Switchable constraint pseudo-range

For more documentation on each of these, see the code.  Hopefully well docstring-ed. :)

ROBOTS is a collection of utilities intended to provide basic functions commonly used by mobile and especially aerial robots. Originally developed as a ROS package, the ROS specific and general-purpose parts of robots have been separated to simplify its usage in monte carlo simulations and offline development and testing of algorithms.

# Installation

pybots is the ros-independent part of robots and is easiest to use if it is installed independently.

clone the repository then

```
cd $robots_directory/pybots
pip install . --user
```

The rosbots utilities can be built conventionally for ROS packages and incorporated into ROS projects via workspace overlays.

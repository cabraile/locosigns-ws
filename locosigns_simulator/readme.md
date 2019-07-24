ABOUT
==============================
ROS/GAZEBO simulation package.

NOTES
==============================
Before launching the simulation for the first time, after building the package, copy the `libPriusHybridPlugin.so` file to `plugins/` (tip: use `find -iname libPriusHybridPlugin.so` for finding it).

FILES
==============================

## Nodes

ROS nodes contained in `node/`.
* `landmark_detector_node.py`: .
* `simulation_node.py`: .
* `velocimeter.py`: .

## Plugins
They provide more functionalities for integrating ROS and Gazebo using URDF files (more [here](http://gazebosim.org/tutorials?tut=ros_gzplugins)).
* `PriusHybridPlugin.*`: the header and the implementation of the plugin that converts `prius_msgs/Control` messages to actions in the simulation. This plugin was obtained from [this](https://github.com/osrf/car_demo) repository.

## Worlds

Gazebo world files in `worlds/`.
* `road.world` contains a straight road.

## Models

Gazebo models to be loaded on the simulated world.

## Launch

Launch files that run all nodes and default parameters.
* `road.launch` launches Gazebo; starts the simulation module ,which defines the path of the robot and update its true state); and runs the simulated sensors' drivers, which provide noisy measurement of the environment. The launch contains more details on its comments. 

TODO
==============================
- [ ] Add dummy models in the simulated world and detecting them (`sim_landmark_detector_node`).
- [ ] Path generator
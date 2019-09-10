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
* `sim_landmark_detector_node.py`: publishes the position, the distance from and the angle between the car and the detected landmark at `/sensor/landmark` as a `locosigns_msg.msg.Landmark` message.
* `sim_control_node.py`: given the true position of the robot, sends the simulator commands to keep the car on the road. Also, publishes the best estimation of odometry, noiseless, at `/sim_sensor/linear_position` as a `locosigns_msg.msg.Scalar` message.
* `sim_velocimeter_node.py`: this node captures the velocity between two time measurements. It publishes the true velocity at `/sim_sensor/velocity_groundtruth` and a noisy measurement (+- 10% of the true velocity) at `/sim_sensor/velocity`, both as `locosigns_msg.msg.Scalar` messages.
* `sim_error_node.py`: measures the difference between the filtered state of the robot (both from `state/filter/odometry` and `state/filter/odometry_corrected`) and the ground truth position provided at `/sim_sensor/linear_position`.

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
- [X] Add dummy models in the simulated world and detecting them (`sim_landmark_detector_node`).
- [X] Path generator

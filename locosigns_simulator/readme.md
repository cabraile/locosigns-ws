ABOUT
==============================
ROS/GAZEBO simulation package.

FILES
==============================

# Nodes

ROS nodes contained in `node/`.
* `landmark_detector_node.py`: .
* `simulation_node.py`: .
* `velocimeter.py`: .

# Worlds

Gazebo world files in `worlds/`.
* `road.world` contains a straight road.

# Models

Gazebo models to be loaded on the simulated world.

# Launch

Launch files that run all nodes and default parameters.
* `road.launch` launches Gazebo; starts the simulation module ,which defines the path of the robot and update its true state); and runs the simulated sensors' drivers, which provide noisy measurement of the environment. The launch contains more details on its comments. 

TODO
==============================
- [ ] Add dummy models in the simulated world and detecting them.
- [ ] Model the simulation to work with the true `x`, `y` and `z` axes, add heading control and make path non-linear.
- [ ] Implement backwards compatibility

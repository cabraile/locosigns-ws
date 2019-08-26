1 ABOUT THE PROJECT
======================
The _Localization Correction using Signs_ (or LoCoSigns) is a method of location correction that fuses odometry and sign information for estimating the position of the robot in a highway.

2 ABOUT THIS REPOSITORY
======================
This workspace contains the packages used in the LoCoSigns project. The standard is to used catkin-like workspaces and everything else for working with ROS.

P.S. The model used for simulation was downloaded from from [this repository](https://github.com/osrf/car_demo). All `prius_*` packages are the simulation model's packages and can be ignored/removed if you do not intend to perform simulation.

3 HOW TO USE
======================


4 NODES
======================


5 TOPICS AND MESSAGES
======================


6 PACKAGES
======================
The list of packages implemented for this project (except the `prius_*` packages) are listed below. A description for each package can be found on their individual directory.
* `locosigns_simulator`: provides a simulated environment, sensors a trajectory for the robot
to move in the simulation.
* `prius_*`: they provide the model, the description and the control of the vehicle.
* `locosigns_msg` : customized messages used in this project.
* `locosigns_filter`: implements the rosnode that performs state estimation.

7 TODO
======================
- [X] Pending list in `locosigns_simulator`

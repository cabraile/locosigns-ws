ABOUT
======================
This workspace contains the packages used in the LoCoSigns project. The standard is to used catkin-like workspaces and everything else for working with ROS.

All `prius_*` packages where downloaded from [this repository](https://github.com/osrf/car_demo).

Requires Gazebo 9 for launching simulation files.

PACKAGES
======================
The list of packages implemented for this project (except the `prius_*` packages) are listed below. A description for each package can be found on their individual directory.
* `locosigns_simulator`: provides a simulated environment, sensors a trajectory for the robot
to move in the simulation.
* `prius_*`: they provide the model, the description and the control of the vehicle.
* `locosigns_msg` : customized messages used in this project.
* `locosigns_filter`: implements the rosnode that performs state estimation.

TODO
======================
- [ ] Pending list in `locosigns_simulator`

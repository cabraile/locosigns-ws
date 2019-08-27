1 ABOUT THE PROJECT
======================
The _Localization Correction using Signs_ (or LoCoSigns) is a method of location correction that fuses odometry and sign information for estimating the position of the robot in a highway. 

2 ABOUT THIS REPOSITORY
======================
This workspace contains the ~prototype~ packages used in the LoCoSigns project. The standard is to used catkin-like workspaces and everything else for working with ROS. 

On this project, all the variables follow the standard unit system (meters, meters per second, radians and so on).

The __TL;DR__ section summarizes the main topics of this readme. However, if you want to read the full version, Section 4 to 7 provide a detailed description of the topics, parameters, messages and usage. Also, make sure you read the Section __TODO__ at the end of this document for checking if the implementation on its state is applicable for your problem.

P.S. The model used for simulation was downloaded from from [this repository](https://github.com/osrf/car_demo). All `prius_*` packages are the simulation model's packages and can be ignored/removed if you do not intend to perform simulation.

3 TL;DR
======================
* __Running the simulation:__ `roslaunch locosigns_simulator run.launch`.
* __Running the filter only:__ `rosrun locosigns_filter filter_node.py`.
* __Subscribed topics of the filter:__ `sim_sensors/imu`, `sim_sensors/speedometer` and `sim_sensors/landmark`.
* __Estimated state topic:__ `state/filter/complete_correction` (published at 100Hz).
* __Params of the filter:__ `stdev_signs`, `stdev_depth`, `stdev_angle`, `accelerometer_noise_density` and `accelerometer_random_walk`.

4 HOW TO RUN THE SIMULATION
======================
Running the ROS/Gazebo simulation along with the filter just requires `roslaunch locosigns_simulator run.launch` to be issued.

5 HOW TO USE THE FILTER
======================
__Running.__ The node that performs filtering `filter_node.py` belongs to the `locosigns_filter` package. So issue `rosrun locosigns_filter filter_node.py` for starting the filter.

__Subscribers.__ It subscribes to the following topics:
* `sim_sensors/imu`, which provides `sensor_msgs/Imu.msg` messages from the data read from the IMU.
* `sim_sensors/speedometer`: which provides `geometry_msgs/TwistStamped.msg` messages from the data read from a speedometer (optional).
* `sim_sensors/landmark`: which provides `locosigns_msgs/Landmark.msg` messages from the signs detector (optional).

The topics that are optional to be provided perform correction to the robot estimation, so if neither are provided the state is estimated using the Imu only (inertial odometry).

__Publishers.__ The complete estimated state (that fuses IMU, speedometer and landmark information) is published at a rate of 100Hz at `state/filter/complete_correction`. This is the recommeded topic for the state estimation. However, for comparison purposes, the following topics  are published as well:
* `state/filter/uncorrected`: the state estimated using inertial odometry only.
* `state/filter/velocity_correction`: velocity correction, but no position correction.
* `state/filter/landmark_correction`: global position correction using detected landmarks, but no correction on velocity.
The states published are of type `locosigns_msgs/State.msg`.

__Params.__ .
* `stdev_signs`: The expected deviation on the signs' true position. Set lower values if you want the localization module to trust more on their indicated position.
* `stdev_depth`: How precise is the measurement of the depth sensor.
* `stdev_angle`: Not exactly the standard deviation. It is the maximum angle in which the signs can be detected.
* `accelerometer_noise_density` and `accelerometer_random_walk`: the additive white noise and the bias of the accelerometer, as detailed [here](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).

6 CUSTOM MESSAGES
======================
* `locosigns_msgs/State.msg`: carries the estimated 1D position (float) and the velocity (float) of the robot, as well the covariance matrix (float[4]) of the estimation (stamped).
* `locosigns_msgs/Landmark.msg`: carries the label (float) of the detected signs, the measured distance (float) from the sensor to the sign and the angle (float) between the heading of the sensor and the center of mass of the signs (stamped). 

7 PACKAGES
======================
The list of packages implemented for this project (except the `prius_*` packages) are listed below. A description for each package can be found on their individual directory.
* `locosigns_simulator`: provides a simulated environment, sensors a trajectory for the robot
to move in the simulation.
* `prius_*`: they provide the model, the description and the control of the vehicle.
* `locosigns_msg` : customized messages used in this project.
* `locosigns_filter`: implements the rosnode that performs state estimation.

TODO
======================
- [X] Pending list in `locosigns_simulator`.
- [ ] Implement correction for the opposite direction as well.
- [ ] C++ implementation of the filter.
- [ ] Consistent prediction even when no IMU data is received.
- [ ] Make initial state, covariance and direction be setted using rosparams.

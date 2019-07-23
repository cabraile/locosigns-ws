ABOUT
================================
This package provides a module that performs a Kalman filter for estimating the
position of a robot on a road.

FILES
================================
* `src/modules/filter.py`: the Kalman filter implementation.
* `src/filter_node.py`: runs the ROS node that receives sensor information, perform state estimation and publishes the state on the topic `/state/filter/position` and its variance on `/state/filter/position_variance`, both as `Scalar` messages from `locosigns_msg`.
* `src/standalone_simulation.py`: runs a simulation for testing the implementation of the filter independent of ROS.
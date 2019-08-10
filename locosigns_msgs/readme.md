ABOUT
================================
Customized messages implemented for this project.

FILES
================================
* `msg/State.msg`: provides the state (position and velocity) of the robot and its covariance.
* `msg/Landmark.msg`: contains a `Header header` (standard header); a `Float64 label` field, in which the position indicated on the landmark is stored; a `float64 measured_distance`, which is the distance between the robot and the projection of the landmark on the road plane; and `float64 heading_angle`, the angle between the horizontal projection of the heading of the robot and the landmark.
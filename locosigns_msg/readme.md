ABOUT
================================
Customized messages implemented for this project.

FILES
================================
* `msg/Scalar.msg`: a simple structure that contains `Header header` (standard header) and `Float64 data`. Basically is sent by sensors that provide only a floating point measurement.
* `msg/Landmark.msg`: contains a `Header header` (standard header); a `Float64 label` field, in which the position indicated on the landmark is stored; a `float64 measured_distance`, which is the distance between the robot and the projection of the landmark on the road plane; and `float64 heading_angle`, the angle between the horizontal projection of the heading of the robot and the landmark.
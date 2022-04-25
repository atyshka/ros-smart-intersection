Smart Intersection Launch
===
Contains the launch files for the main project simulation and an individual intersection. The main project can be launch with:

  `roslaunch smart_intersection_launch smart_intersection.launch`

Individual intersections can be added by including the `intersection.launch` file with the following parameters:

  * `intersection_id`: a unique identifier to keep each intersection in its own private namespace
  * `pose_x`: the x coordinate of the center of the intersection in gazebo
  * `pose_y`: the y coordinate of the center of the intersection in gazebo
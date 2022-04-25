Pose Follower Ackermann
===
This node implements a control node that runs in the audibot stack that tracks the distance of the audibot from an intersection, then 
communicates with the intersection to request a path on approach. Once received, it requests control of a mux to pass its cmd_vel twist 
messages to the vehicle level contoller. Once it has control, it will follow the intersection's generated path using its integrated PID
controllers.

Example
---
Use the following command to generate a test path:

rostopic pub /path_req smart_intersection/PathRequest "header: auto
vehicle_id: 0
direction: 2
pose:
  header: auto
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
velocity: 0.0"
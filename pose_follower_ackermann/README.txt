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
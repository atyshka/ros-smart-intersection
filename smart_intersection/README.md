Smart Intersection
===

This node implements the intersection object node and associated messages. In the smart intersection project, the vehicle will send the PathRequest
message when approaching the intersection. At that point, this node will generate a path for the vehicle to proceed safely through and send it
with the GuidedPath message type.
#Standardized message for the communication of intersection geometry to the vehicle

std_msgs/Header header
int64 vehicle_id

uint8 UP = 0
uint8 DOWN = 1
uint8 LEFT = 2
uint8 RIGHT = 3

uint8 direction
geometry_msgs/PoseStamped pose
float64 velocity

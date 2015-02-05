# plan feedtack types
uint8 PLAN_UNKNOWN = 0
uint8 NO_PLAN_FOUND = 1
uint8 PLAN_FOUND = 2
uint8 EXECUTION_FAILED = 3
uint8 EXECUTION_SUCCEED = 4

# Waypoint Information for use in Affordance Templates

# the abstract ID of the end effector
uint8 id

# the name of the end effector
string end_effector_name

# the number of waypoints for this the end effector
uint8 num_waypoints

# the current index of the end effector in a trajectory
int8 waypoint_index

# the last wp index of the trajectory
int8 final_waypoint_idx

# the plan status
uint8 plan_status

# waypoint info
#geometry_msgs/PoseStamped[] waypoint_poses



int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
# Field default value 255
int8 robot_side

# The taskspace trajectory information.
controller_msgs/SE3TrajectoryMessage taskspace_trajectory_message

# The jointspace trajectory information.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory_message



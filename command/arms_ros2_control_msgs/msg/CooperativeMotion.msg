# Dual-arm rigid-relative-pose cooperative motion request.
# All poses are expressed in frame_id.
geometry_msgs/Pose left_start
geometry_msgs/Pose right_start
geometry_msgs/Pose left_goal
geometry_msgs/Pose right_goal

string frame_id
string ik_type

# Requested duration. It is extended when auto_extend_duration is true and
# the requested value violates a velocity or acceleration limit.
float64 duration
bool auto_extend_duration true

# Geometric sampling is independent of duration.
float64 max_translation_step 0.005
float64 max_rotation_step 0.01

# Cartesian limits apply to both end effectors.
float64 max_linear_velocity 0.25
float64 max_linear_acceleration 1.0
float64 max_angular_velocity 0.5
float64 max_angular_acceleration 1.0

# Joint velocity comes from URDF and is multiplied by this scale.
# Acceleration is a common per-joint limit because URDF does not provide one.
float64 joint_velocity_scale 1.0
float64 max_joint_acceleration 1.0

# Requested start pose must agree with FK of the current joint state.
float64 start_position_tolerance
float64 start_orientation_tolerance

# The left-to-right transform at the goal must agree with the one at the start.
float64 relative_position_tolerance
float64 relative_orientation_tolerance

# IK/FK validation settings.
int32 ik_max_iterations
float64 ik_tolerance

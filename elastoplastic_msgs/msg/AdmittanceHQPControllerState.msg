std_msgs/Header header

control_msgs/AdmittanceControllerState admittance_state

float64[] z
float64[] zp
float64 reset_buffer_state
float64 reset_buffer_fill

int8 mode
int8 MODE_ELASTIC = 0
int8 MODE_PLASTIC = 1
int8 MODE_RESTORE = 2

geometry_msgs/Pose  cart_ref_pose
geometry_msgs/Twist cart_ref_twist

geometry_msgs/Pose  cart_actual_cmd_pose
geometry_msgs/Twist cart_actual_cmd_twist
geometry_msgs/Twist cart_actual_cmd_acc

geometry_msgs/Pose  cart_actual_pose
geometry_msgs/Twist cart_actual_twist

float64[] tau_cmd

float64[] gravity

sensor_msgs/JointState joint_reference

int64 cycle_time_nanoseconds

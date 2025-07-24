std_msgs/Header header

control_msgs/AdmittanceControllerState admittance_state

float64[] z
float64[] zp
float64 reset_buffer_state

int8 mode
int8 MODE_ELASTIC = 0
int8 MODE_PLASTIC = 1
int8 MODE_RESTORE = 2

geometry_msgs/Pose cart_ref_pose
geometry_msgs/Twist cart_ref_twist

geometry_msgs/Pose cart_computed_ref_pose
geometry_msgs/Twist cart_computed_ref_twist
geometry_msgs/Twist cart_computed_ref_acc

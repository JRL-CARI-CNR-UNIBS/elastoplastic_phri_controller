std_msgs/Header header

control_msgs/AdmittanceControllerState admittance_state
geometry_msgs/Wrench virtual_force

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

sensor_msgs/JointState joint_reference

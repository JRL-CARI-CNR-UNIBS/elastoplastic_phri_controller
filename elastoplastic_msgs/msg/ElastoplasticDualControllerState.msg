std_msgs/Header header

int8 SIDE_LEFT = 0
int8 SIDE_RIGHT = 1

control_msgs/AdmittanceControllerState admittance_state # Cartesian-shared frame related values
geometry_msgs/Wrench virtual_force # Force computed by the admittance controller
geometry_msgs/Wrench[2] wrenches # wrenched read from ft-sensors

float64[] z
float64[] zp
float64 reset_buffer_state

int8 mode
int8 MODE_ELASTIC = 0
int8 MODE_PLASTIC = 1
int8 MODE_RESTORE = 2

geometry_msgs/Pose  cart_ref_pose
geometry_msgs/Twist cart_ref_twist

geometry_msgs/Pose  cart_actual_shared_pose
geometry_msgs/Twist cart_actual_shared_twist

geometry_msgs/Pose[2]  cart_actual_tool_pose
geometry_msgs/Twist[2] cart_actual_tool_twist

geometry_msgs/Pose[2]  cart_actual_cmd_pose
geometry_msgs/Twist[2] cart_actual_cmd_twist
geometry_msgs/Twist[2] cart_actual_cmd_acc

sensor_msgs/JointState joint_reference

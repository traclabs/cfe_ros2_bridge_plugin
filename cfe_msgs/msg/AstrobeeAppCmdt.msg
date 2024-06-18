# cFE NAME: AstrobeeAppCmd_t
# ROS topic: astrobeeappcmd_t
# Command message
std_msgs/Header header
CFEMSGCommandHeader cmd_header
float32 x
float32 y
float32 z
float32 qx
float32 qy
float32 qz
float32 qw
string cmd_name
string cmd_id
string cmd_src
string cmd_origin
string subsys_name
string frame

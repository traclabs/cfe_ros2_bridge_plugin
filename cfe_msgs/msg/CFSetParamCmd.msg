# cFE NAME: CF_SetParamCmd
# ROS topic: cf_setparamcmd
# Command message
std_msgs/Header header
CFEMSGCommandHeader cmd_header
uint32 value
uint8 key
uint8 chan_num
uint8[] spare

# cFE NAME: CF_WriteQueueCmd
# ROS topic: cf_writequeuecmd
# Command message
std_msgs/Header header
CFEMSGCommandHeader cmd_header
uint8 type
uint8 chan
uint8 queue
uint8 spare
string filename

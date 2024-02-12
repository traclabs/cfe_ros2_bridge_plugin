# cFE NAME: SCH_LAB_StateEntry_t
# ROS topic: sch_lab_stateentry_t
# Command message
std_msgs/Header header
CFEMSGCommandHeader command_header
uint32 packet_rate
uint16[] message_buffer
uint32[] counter
uint16 payload_length
uint16 spare0

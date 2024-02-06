# cFE NAME: CF_TxFileCmd
# ROS topic: cf_txfilecmd
# Command message
std_msgs/Header header
CFEMSGCommandHeader cmd_header
uint8 cfdp_class
uint8 keep
uint8 chan_num
uint8 priority
uint32 dest_id
string src_filename
string dst_filename

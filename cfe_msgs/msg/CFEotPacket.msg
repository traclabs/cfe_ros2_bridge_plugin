# cFE NAME: CF_EotPacket
# ROS topic: cf_eotpacket
# Telemetry message
std_msgs/Header header
int32 seq
CFEMSGTelemetryHeader tlm_header
uint32 seq_num
uint32 channel
uint32 direction
uint32 state
uint32 txn_stat
uint32 src_eid
uint32 peer_eid
uint32 fsize
uint32 crc_result
CFTxnFilenames fnames

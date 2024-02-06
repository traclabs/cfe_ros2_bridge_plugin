# cFE NAME: CF_HkPacket
# ROS topic: cf_hkpacket
# Telemetry message
std_msgs/Header header
int32 seq
CFEMSGTelemetryHeader tlm_header
CFHkCmdCounters counters
uint8[] spare
CFHkChannelData[] channel_hk

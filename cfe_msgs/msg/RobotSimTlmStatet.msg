# cFE NAME: RobotSimTlmState_t
# ROS topic: robotsimtlmstate_t
# Telemetry message
std_msgs/Header header
int32 seq
CFEMSGTelemetryHeader tlm_header
RobotSimSSRMSt joints
float32 kp
float32[] errors

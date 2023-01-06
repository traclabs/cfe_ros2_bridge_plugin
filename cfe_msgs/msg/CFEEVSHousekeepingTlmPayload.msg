# cFE NAME: CFE_EVS_HousekeepingTlm_Payload
# ROS topic: cfe_evs_housekeepingtlm_payload
uint8 command_counter
uint8 command_error_counter
uint8 message_format_mode
uint8 message_trunc_counter
uint8 unregistered_app_counter
uint8 output_port
uint8 log_full_flag
uint8 log_mode
uint16 message_send_counter
uint16 log_overflow_counter
uint8 log_enabled
uint8 spare
uint8 spare_1
uint8 spare_2
CFEEVSAppTlmData[] app_data

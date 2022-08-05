# cFE NAME: Telemetry Output LAB
# FILE: to_lab_msg
uint8 TO_NOP_CC=0
uint8 TO_RESET_STATUS_CC=1
uint8 TO_ADD_PKT_CC=2
uint8 TO_SEND_DATA_TYPES_CC=3
uint8 TO_REMOVE_PKT_CC=4
uint8 TO_REMOVE_ALL_PKT_CC=5
uint8 TO_OUTPUT_ENABLE_CC=6

uint8 cmd_id

ToAddPktCc to_add_pkt_cc
ToRemovePktCc to_remove_pkt_cc
ToOutputEnableCc to_output_enable_cc

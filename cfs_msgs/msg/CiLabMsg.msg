# cFE NAME: Command Ingest LAB
# FILE: ci_lab_msg
uint8 CI_NOOP_CC=0
uint8 CI_RESET_COUNTERS_CC=1
uint8 CI_MODIFY_PDU_FILESIZE_CC=2
uint8 CI_CORRUPT_PDU_CHECKSUM_CC=3
uint8 CI_DROP_PDUS_CC=4
uint8 CI_CAPTURE_PDUS_CC=5
uint8 CI_STOP_PDU_CAPTURE_CC=6

uint8 cmd_id

CiModifyPduFilesizeCc ci_modify_pdu_filesize_cc
CiDropPdusCc ci_drop_pdus_cc
CiCapturePdusCc ci_capture_pdus_cc

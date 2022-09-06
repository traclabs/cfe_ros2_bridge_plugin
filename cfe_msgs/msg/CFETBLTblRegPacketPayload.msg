# cFE NAME: CFE_TBL_TblRegPacket_Payload
# ROS topic: cfe_tbl_tblregpacket_payload
uint32 size
uint32 crc
uint32 active_buffer_addr
uint32 inactive_buffer_addr
uint32 validation_func_ptr
CFETIMESysTime time_of_last_update
uint32 file_create_time_secs
uint32 file_create_time_sub_secs
bool table_loaded_once
bool load_pending
bool dump_only
bool double_buffered
string name
string last_file_loaded
string owner_app_name
bool critical
uint8 byte_align
uint16 spare_end

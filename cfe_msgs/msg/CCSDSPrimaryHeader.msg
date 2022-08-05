# packet identifier word (stream ID)
uint8[] stream_id  
#  bits  shift   ------------ description ---------------- 
#  0x07FF    0  : application ID                            
#  0x0800   11  : secondary header: 0 = absent, 1 = present 
#  0x1000   12  : packet type:      0 = TLM, 1 = CMD        
#  0xE000   13  : CCSDS version:    0 = ver 1, 1 = ver 2    
#

# packet sequence word 
uint8[] sequence
#  bits  shift   ------------ description ---------------- 
#  0x3FFF    0  : sequence count                            
#  0xC000   14  : segmentation flags:  3 = complete packet  

# packet length word
uint8[] length
#  bits  shift   ------------ description ---------------- 
#  0xFFFF    0  : (total packet length) - 7                 
